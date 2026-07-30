#include "pwm.h"

#include "../../firmware_logic.h"
#include "hardware/clocks.h"

Pwm::Pwm(int pin, int freq, float max, float min)
    : pin(pin),
      freq(freq),
      slice(-1),
      channel(-1),
      wrap(0),
      dividerInteger(0),
      dividerFraction(0),
      actualFrequency(0) {
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

void Pwm::forceLow() {
    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

bool Pwm::init() {
    initialized = false;
    duty = 0;
    firmware::PwmTiming timing =
        firmware::calculate_pwm_timing(clock_get_hz(clk_sys), freq);
    if (!timing.valid) {
        forceLow();
        return false;
    }

    wrap = timing.wrap;
    dividerInteger = timing.divider_integer;
    dividerFraction = timing.divider_fraction_16;
    actualFrequency = static_cast<float>(timing.actual_frequency_hz);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);
    pwm_set_clkdiv_int_frac(slice, dividerInteger, dividerFraction);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, 0);
    initialized = true;
    return true;
}

bool Pwm::restore() {
    if (!initialized) {
        forceLow();
        return false;
    }
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice, dividerInteger, dividerFraction);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, true);
    duty = 0;
    return true;
}

void Pwm::write(float duty) {
    if (!initialized) {
        forceLow();
        this->duty = 0;
        return;
    }
    if (duty > MAX_DUTY) {
        duty = MAX_DUTY;
    } else if (duty < MIN_DUTY) {
        duty = MIN_DUTY;
    }
    pwm_set_chan_level(
        slice, channel,
        static_cast<uint16_t>(static_cast<float>(wrap) * duty));
    pwm_set_enabled(slice, 1);
    this->duty = duty;
}

float Pwm::read() {
    return duty;
}

bool Pwm::isInitialized() const {
    return initialized;
}

uint16_t Pwm::getWrap() const {
    return wrap;
}

float Pwm::getActualFrequency() const {
    return actualFrequency;
}
