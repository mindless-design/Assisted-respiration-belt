#include "photon_adc_dma.h"

// Class constructor. This connects the initaization variables to the private varaibles defined in the header.
ADCDMA_config::ADCDMA_config(int pin, uint16_t *buf, size_t bufSize) : pin(pin), buf(buf), bufSize(bufSize) {
}

//Class deconstrctor. Not used in this case.
ADCDMA_config::~ADCDMA_config() {
}

void ADCDMA_config::start(size_t freqHZ) {

    // Using Dual ADC Regular Simultaneous DMA Mode 1

    // Using Timer3. To change timers, make sure you edit all of:
    // RCC_APB1Periph_TIM3, TIM3, ADC_ExternalTrigConv_T3_TRGO

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Set the pin as analog input
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    HAL_Pin_Mode(pin, AN_INPUT);

    // Enable the DMA Stream IRQ Channel
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 60000000UL = 60 MHz Timer Clock = HCLK / 2
    // Even low audio rates like 8000 Hz will fit in a 16-bit counter with no prescaler (period = 7500)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (60000000UL / freqHZ) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T3_TRGO
    TIM_Cmd(TIM3, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // DMA2 Stream0 channel0 configuration
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr =  0x40012308; // CDR_ADDRESS; Packed ADC1, ADC2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = bufSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    // Don't enable DMA Stream Half / Transfer Complete interrupt
    // Since we want to write out of loop anyway, there's no real advantage to using the interrupt, and as
    // far as I can tell, you can't set the interrupt handler for DMA2_Stream0 without modifying
    // system firmware because there's no built-in handler for it.
    // DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

    DMA_Cmd(DMA2_Stream0, ENABLE);

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1 configuration
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC2 configuration - same
    ADC_Init(ADC2, &ADC_InitStructure);

    //
    Hal_Pin_Info* PIN_MAP = HAL_Pin_Map();
    ADC_RegularChannelConfig(ADC1, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);
    // Serial.printlnf("using pin %d ADC channel %u", pin, PIN_MAP[pin].adc_channel);

    // Enable DMA request after last transfer (Multi-ADC mode)
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // Enable ADCs
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_SoftwareStartConv(ADC1);
}

void ADCDMA_config::stop() {
    // Stop the ADC
    ADC_Cmd(ADC1, DISABLE);
    ADC_Cmd(ADC2, DISABLE);

    DMA_Cmd(DMA2_Stream0, DISABLE);

    // Stop the timer
    TIM_Cmd(TIM3, DISABLE);
}