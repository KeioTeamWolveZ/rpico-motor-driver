#pragma once

#include <cstdint>

#include "hardware/pio.h"
#include "pico/time.h"

class PioServoOutput {
   public:
    explicit PioServoOutput(uint gpio);

    bool enable();
    bool write_pulse_us(uint32_t pulse_us);
    void disable();

    bool is_enabled() const;
    bool is_pulsing() const;
    bool has_feed_fault() const;
    const char* last_error() const;
    uint32_t applied_pulse_us() const;
    int state_machine() const;

   private:
    static bool frame_timer_callback(repeating_timer_t* timer);
    bool configure_state_machine();
    void force_gpio_low();
    void release_resources();
    bool queue_pulse();
    void fail_closed(const char* error);

    uint gpio_;
    PIO pio_;
    int sm_;
    int program_offset_;
    bool enabled_;
    volatile bool pulsing_;
    bool frame_timer_started_;
    volatile bool feed_fault_;
    volatile uint32_t pulse_program_count_;
    uint32_t applied_pulse_us_;
    double pio_tick_hz_;
    const char* last_error_;
    repeating_timer_t frame_timer_;
};
