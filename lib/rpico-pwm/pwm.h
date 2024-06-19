#pragma once
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Pwm {
   public:
    Pwm(int pin, int freq=1000, float max=1, float min=0);
    void init();
    void write(float val);
    float read();

   private:
    int pin;
    int freq;
    float MAX_DUTY;
    float MIN_DUTY;
    int slice;
    int channel;
    float duty;

};
