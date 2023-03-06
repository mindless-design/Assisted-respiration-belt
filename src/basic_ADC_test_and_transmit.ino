#include <math.h>
#include "photon_adc_dma.h"
#include "photon_fft.h"
#include "FIR_coeffs.h"

// This is the pin the strain gauge and current source is connected to.
const int SAMPLE_PIN = A0;

// These are the digital/analog write pins that control the peltier elements.
const int HOT_PELTIER_PIN = D0;
const int PELTIER_ENABLE_PIN = D1;
const int COLD_PELTIER_PIN = D2;

// 512 is a good size for this buffer. This is the number of samples; the number of bytes is twice
// this, but only half the buffer is handled a time, and then each pair of samples is averaged.
const size_t SAMPLE_BUF_SIZE = 2048;
uint16_t samples[SAMPLE_BUF_SIZE];

// We want the sample rate of the ADC to be as close to a multiple of 1/60 and of radix 2 as possible.
// This is so that we can average over a single (or multiple) period(s) of 60 Hz radiation to cancel out line frequency interference.
const double SAMPLE_RATE_AFTER_DECIMATION = 4;

// The decimation factor defines how many extra samples per second we will take then filter/downsample (decimate).
const int DECIMATION_FACTOR = 512;

// The sample rate of the ADCs
const int SAMPLE_RATE = SAMPLE_RATE_AFTER_DECIMATION*DECIMATION_FACTOR;

// This is the interval that we want to calculate the FFT over in seconds.
// Make sure that this value is of radix 2 (2^x).  
const int FFT_WINDOW_PERIOD = 32;

// This is how much we want each window to overlap with the last one in seconds (also how often the cloud variable is updated).
const int FFT_WINDOW_OVERLAP = 10;

// Variables for decimation and fft computation:
const double fft_window_data_size = SAMPLE_RATE_AFTER_DECIMATION*FFT_WINDOW_PERIOD;
const int fft_overlap_window_data_size = SAMPLE_RATE_AFTER_DECIMATION*FFT_WINDOW_OVERLAP;

// The size of each frequency division in Hz
const double BIN_SIZE = SAMPLE_RATE_AFTER_DECIMATION/fft_window_data_size;

// Since we want to ignore the DC componenet and ultra low frequencies, we
// cut off a number of 'bins' from the beginnning of the spectrum.
const int BINS_TO_BE_CUT_OFF = 2;

// Data is stored in these vectors for the FFT to do it's magic on.
std::vector<double> fft_window_data_real;
std::vector<double> fft_window_data_imag;


// Data is copied into this vector so that we can save previous data chunks.
std::vector<double> fft_window_data_copy;

double analog_value_after_filter = -1;

double current_value_of_single_sided_spectrum;

double respiration_rate_per_minute = -1;

ADCDMA_config adcDMA(SAMPLE_PIN, samples, SAMPLE_BUF_SIZE);

// Frequency and intensity of the peltier elements.
int frequency_in_milihertz = 0;
int hot_intensity = 0;
int cold_intensity = 0;

// The frequency of the heating and cooling peltier elements is controlled by a software timer with a period corresponding to the frequency given by the facilitator.

int default_period = 4000;

int hot_PWM_duty_cycle = 0; // Duty cycle set by an 8 bit register. Values from 0 - 255.
int cold_PWM_duty_cycle = 0;

int hot_PWM_frequency = 10; // Frequency of the PWM signal. From 1 Hz to really fast.
int cold_PWM_frequency = 10;

Timer peltier_timer((default_period/4), switch_peltiers);

char state = 'D'; // H = Hot on. C = Cold on. I = Hot off. D = Cold off.

void switch_peltiers() {

  switch (state) {
    case 'D' : 
      analogWrite(HOT_PELTIER_PIN, 0);
      analogWrite(COLD_PELTIER_PIN, 0);

      state = 'H';
      break;
    
    case 'I' :
      analogWrite(HOT_PELTIER_PIN, 0);
      analogWrite(COLD_PELTIER_PIN, 0);

      state = 'C';
      break;
    
    case 'H' :
      analogWrite(HOT_PELTIER_PIN, hot_PWM_duty_cycle, hot_PWM_frequency);

      state = 'I';
      break;

    case 'C' :
      analogWrite(COLD_PELTIER_PIN, cold_PWM_duty_cycle, cold_PWM_frequency);

      state = 'D';
      break;
  }
}

// This is the code that recieves the data from the facilitator to change the intensity or frequency of the peltier elements.
// For the peltier element driving, we want the user to submit two values to the photon: the frequency in mHz and an intensity value on a scale of 1 - 10.
size_t space_position = 0;

/* old freq/intensity function

int set_peltier_frequency_and_intensity (String command) {
  if ((space_position = command.indexOf(" ")) != command.length() - 1) { // This section of the code does not work. The comand.find() function does not exsist.
      frequency_in_milihertz = atoi(command.substring(0, space_position)); // The command.substring() also does not exsist.
      intensity = atoi(command.substring(space_position, command.length()-1));
  } else {
    Serial.print("No space in command string. Check input: ");
    Serial.println(command);

    return -1; // Failed function call.
  }

  if (frequency_in_milihertz <= 500 && frequency_in_milihertz >= 33 && intensity >= 0 && intensity <= 10) {
    int timer_period_in_ms = 250000/frequency_in_milihertz;
    peltier_timer.changePeriod(timer_period_in_ms);
    // change_peltier_intensity(intensity);

    return 1; // Success
  }
  return 0;
}
*/

int set_peltier_frequency (String command) {
  frequency_in_milihertz = atoi(command);
  if (frequency_in_milihertz <= 500 && frequency_in_milihertz >= 33) {
    int timer_period_in_ms = 250000/frequency_in_milihertz;
    peltier_timer.changePeriod(timer_period_in_ms);
    return frequency_in_milihertz; //success
  }
  else {
    return -1;
  }
}

//intensity functions currently set to take any int between 0-255, will change to 0-10 scale
int set_hot_peltier_intensity (String command) {
  hot_intensity = atoi(command);
  if (hot_intensity >= 0 && hot_intensity <= 255) { //<= 10) {
   hot_PWM_duty_cycle = hot_intensity; //hot_intensity * 13;
   return hot_intensity; //Success
  }
  else {
    return -1;
  }
}

int set_cold_peltier_intensity (String command) {
  cold_intensity = atoi(command);
  if (cold_intensity >= 0 && cold_intensity <= 255) { // <= 10) {
   cold_PWM_duty_cycle = cold_intensity; //cold_intensity * 13;
   return cold_intensity; //Success
  }
  else {
    return -1;
  }
}

// setup() runs once, when the device is first turned on.
void setup() {

  Serial.begin(9600);

  // Configure the ADC and DMA handler to move samples into a circular buffer.
	adcDMA.start(SAMPLE_RATE);

	// Setup for push to cloud
	Particle.variable("respiration", respiration_rate_per_minute);

  // Setup for recieving the respiration feedback data.
  Particle.function("frequency", set_peltier_frequency);
  Particle.function("hot intensity", set_hot_peltier_intensity);
  Particle.function("cold intensity", set_cold_peltier_intensity);

  pinMode(HOT_PELTIER_PIN, OUTPUT);
  pinMode(COLD_PELTIER_PIN, OUTPUT);
  pinMode(PELTIER_ENABLE_PIN, OUTPUT);

  digitalWrite(PELTIER_ENABLE_PIN, HIGH);
  peltier_timer.start();
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  
  // Set the pointer to the correct element in the circular buffer that DMA is writing to by 
  // checking the half and full flags configured in ADCDMA_config.

	uint16_t *samples_buffer = NULL;


	if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_HTIF0)) {
	  DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
	  samples_buffer = samples;
	}
	if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0)) {
	  DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
	  samples_buffer = &samples[SAMPLE_BUF_SIZE/2-1];
	}


  if (samples_buffer != NULL) {


    double filtered_sample = 0;
    double sum = 0;

    // Since the ADCs are sampling simulaneously, the samples from each need to be averaged. Then
    // the FIR filter is applied and only 1 of every "DECIMATION_FACTOR" samples is saved.

    for (size_t ii = 0, jj = 0; ii < SAMPLE_BUF_SIZE/2; ii += 2, jj++) {
      
      sum = (double)samples_buffer[ii]+(double)samples_buffer[ii+1];

      filtered_sample += (sum/2)*FILTER_COEFFS[jj];
    }

    Serial.print("filtered sample: ");
    Serial.println(filtered_sample, 4);

    fft_window_data_real.push_back(filtered_sample);

    if (fft_window_data_real.size() == fft_window_data_size) {
       // create fft instance.
      Fft fft;

      fft_window_data_imag.assign(fft_window_data_size, 0);
      fft_window_data_copy = fft_window_data_real;

      fft.transformRadix2(fft_window_data_real, fft_window_data_imag);

      // We then calculate the relative magnitudes of the single sided spectrum by cutting the spectrum in half,
      // calculating the realtive magnitude of the result (real^2+imag^2). If we were interested
      // in absolute magnitudes, we would take the square root then multiply by two - to account for the symmetry - 
      // and multiply by 1/sqrt(fft_window_data_size) - to account for the number of samples in the spectrum.

      // We then find the index of the maximum peak in the spectrum and mutiply it with the BIN_SIZE.

      double largest_peak = 0;
      double respiration_frequency = 0;

      for (size_t i = BINS_TO_BE_CUT_OFF; i < fft_window_data_size/2 + 1; i++) {
        current_value_of_single_sided_spectrum = fft_window_data_real[i]*fft_window_data_real[i]+fft_window_data_imag[i]*fft_window_data_imag[i];
        
        Serial.print("Current value of spectrum: ");
        Serial.println(current_value_of_single_sided_spectrum, 4);

        if (i == BINS_TO_BE_CUT_OFF) {
          largest_peak = current_value_of_single_sided_spectrum;
          respiration_frequency = BIN_SIZE*i;
        } else {
          if (current_value_of_single_sided_spectrum > largest_peak) {
            largest_peak = current_value_of_single_sided_spectrum;
            respiration_frequency = BIN_SIZE*i;
          }
        }
      }

      respiration_rate_per_minute = respiration_frequency*60;

      Particle.publish("respiration rate", String(respiration_rate_per_minute));
      //Particle.variable("respiration rate", respiration_rate_per_minute);
            
      Serial.print("respiration ");
      Serial.println(respiration_rate_per_minute, 4);

      // Re-assign the time-domain data after the FFT and remove the earliest data for the overlap.
      fft_window_data_copy.erase(fft_window_data_copy.begin(), fft_window_data_copy.begin() + fft_overlap_window_data_size);
      fft_window_data_real = fft_window_data_copy;
    }
  }
}