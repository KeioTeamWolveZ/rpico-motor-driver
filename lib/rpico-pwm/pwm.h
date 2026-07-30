#pragma once
#include <stdint.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Pwm {
   public:
    Pwm(int pin, int freq=1000, float max=1, float min=0);
    bool init();
    bool restore();
    void write(float val);
    float read();
    bool isInitialized() const;
    uint16_t getWrap() const;
    float getActualFrequency() const;

   private:
    void forceLow();

    int pin;
    int freq;
    float MAX_DUTY;
    float MIN_DUTY;
    int slice;
    int channel;
    uint16_t wrap;
    uint8_t dividerInteger;
    uint8_t dividerFraction;
    float actualFrequency;
    float duty = 0;
    bool initialized = false;

};
