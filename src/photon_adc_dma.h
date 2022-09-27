#include "Particle.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"

class ADCDMA_config {
    public:
        ADCDMA_config(int pin, uint16_t *buf, size_t bufSize);
        virtual ~ADCDMA_config();
        void start(size_t freqHZ);
        void stop();

    private:
        int pin;
        uint16_t *buf;
        size_t bufSize;
};