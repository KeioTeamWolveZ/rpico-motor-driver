#include "pwm.h"

Pwm::Pwm(int pin, int freq, float max, float min)
    : pin(pin), freq(freq) {
    if (max > 1) {
        max = 1;
    } else if (max < 0) {
        max = 0;
    }
    if (min > 1) {
        min = 1;
    } else if (min < 0) {
        min = 0;
    }
    MAX_DUTY = max;
    MIN_DUTY = min;
}

void Pwm::init() {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);
    pwm_set_clkdiv(slice, (float)125E6 / (2048 * freq));
    pwm_set_wrap(slice, 2047);
}

void Pwm::write(float duty) {
    if (duty > MAX_DUTY) {
        duty = MAX_DUTY;
    } else if (duty < MIN_DUTY) {
        duty = MIN_DUTY;
    }
    pwm_set_chan_level(slice, channel, (uint16_t)(2047 * duty));
    pwm_set_enabled(slice, 1);
    this->duty = duty;
}

float Pwm::read() {
    return duty;
}