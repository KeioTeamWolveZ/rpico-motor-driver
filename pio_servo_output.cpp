#include "pio_servo_output.h"

#include <cmath>

#include "firmware_logic.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "servo_pulse.pio.h"

PioServoOutput::PioServoOutput(uint gpio)
    : gpio_(gpio),
      pio_(pio2),
      sm_(-1),
      program_offset_(-1),
      enabled_(false),
      pulsing_(false),
      frame_timer_started_(false),
      feed_fault_(false),
      pulse_program_count_(0),
      applied_pulse_us_(0),
      pio_tick_hz_(0.0),
      last_error_("NONE"),
      frame_timer_{} {}

void PioServoOutput::force_gpio_low() {
    gpio_init(gpio_);
    gpio_set_function(gpio_, GPIO_FUNC_SIO);
    gpio_set_dir(gpio_, GPIO_OUT);
    gpio_put(gpio_, 0);
}

bool PioServoOutput::configure_state_machine() {
    uint32_t clock_hz = clock_get_hz(clk_sys);
    firmware::PioClockTiming timing =
        firmware::calculate_pio_clock_timing(
            clock_hz, firmware::kServoPioTickHz);
    if (!timing.valid) {
        last_error_ = "INVALID_PIO_CLOCK";
        return false;
    }
    pio_tick_hz_ = timing.actual_frequency_hz;

    pio_sm_config config =
        servo_pulse_program_get_default_config(
            static_cast<uint>(program_offset_));
    sm_config_set_sideset_pins(&config, gpio_);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac(
        &config,
        timing.divider_integer,
        timing.divider_fraction_256);

    int init_result = pio_sm_init(
        pio_,
        static_cast<uint>(sm_),
        static_cast<uint>(program_offset_),
        &config);
    if (init_result != PICO_OK) {
        last_error_ = "PIO_SM_INIT_FAILED";
        return false;
    }

    pio_gpio_init(pio_, gpio_);
    if (pio_sm_set_consecutive_pindirs(
            pio_, static_cast<uint>(sm_), gpio_, 1, true) != PICO_OK) {
        last_error_ = "PIO_PIN_DIRECTION_FAILED";
        return false;
    }
    pio_sm_set_enabled(pio_, static_cast<uint>(sm_), false);
    pio_sm_clear_fifos(pio_, static_cast<uint>(sm_));
    pio_sm_restart(pio_, static_cast<uint>(sm_));
    pio_sm_set_pins_with_mask(
        pio_, static_cast<uint>(sm_), 0, 1u << gpio_);
    return true;
}

bool PioServoOutput::enable() {
    if (enabled_) {
        return true;
    }

    force_gpio_low();
    last_error_ = "NONE";
    feed_fault_ = false;
    applied_pulse_us_ = 0;
    pulse_program_count_ = 0;

    // Qenc currently uses pio0/sm0 and pio1/sm0 without cooperative claiming.
    // Reserve only PIO2 so servo allocation cannot collide with either encoder.
    if (pio_get_gpio_base(pio_) != 0) {
        last_error_ = "PIO2_GPIO_BASE_CONFLICT";
        return false;
    }

    sm_ = pio_claim_unused_sm(pio_, false);
    if (sm_ < 0) {
        last_error_ = "PIO2_SM_UNAVAILABLE";
        return false;
    }
    if (!pio_can_add_program(pio_, &servo_pulse_program)) {
        last_error_ = "PIO2_PROGRAM_SPACE_UNAVAILABLE";
        release_resources();
        return false;
    }
    program_offset_ = pio_add_program(pio_, &servo_pulse_program);
    if (program_offset_ < 0) {
        last_error_ = "PIO2_PROGRAM_ADD_FAILED";
        release_resources();
        return false;
    }
    if (!configure_state_machine()) {
        const char* error = last_error_;
        release_resources();
        force_gpio_low();
        last_error_ = error;
        return false;
    }

    enabled_ = true;
    pulsing_ = false;
    // The state machine remains disabled and GPIO5 remains LOW until the first
    // validated command is accepted.
    return true;
}

bool PioServoOutput::queue_pulse() {
    if (!enabled_ || sm_ < 0 ||
        pio_sm_is_tx_fifo_full(pio_, static_cast<uint>(sm_))) {
        return false;
    }
    pio_sm_put(
        pio_, static_cast<uint>(sm_), pulse_program_count_);
    return true;
}

bool PioServoOutput::frame_timer_callback(repeating_timer_t* timer) {
    PioServoOutput* self =
        static_cast<PioServoOutput*>(timer->user_data);
    if (self == nullptr || !self->pulsing_) {
        return false;
    }
    if (!self->queue_pulse()) {
        self->feed_fault_ = true;
        self->pulsing_ = false;
        return false;
    }
    return true;
}

bool PioServoOutput::write_pulse_us(uint32_t pulse_us) {
    if (!enabled_) {
        last_error_ = "SERVO_NOT_ENABLED";
        return false;
    }
    if (pulse_us < firmware::kServoMinPulseUs ||
        pulse_us > firmware::kServoMaxPulseUs ||
        pio_tick_hz_ <= 0.0) {
        last_error_ = "INVALID_PULSE_WIDTH";
        return false;
    }

    uint32_t pulse_ticks = static_cast<uint32_t>(
        std::llround(pulse_us * pio_tick_hz_ / 1000000.0));
    if (pulse_ticks < 2) {
        last_error_ = "INVALID_PULSE_TICKS";
        return false;
    }
    pulse_program_count_ =
        firmware::pio_pulse_program_count(pulse_ticks);
    applied_pulse_us_ = pulse_us;
    feed_fault_ = false;

    if (pulsing_) {
        // The new width is picked up by the next 20 ms frame callback.
        last_error_ = "NONE";
        return true;
    }

    if (!add_repeating_timer_us(
            -static_cast<int64_t>(firmware::kServoFrameUs),
            frame_timer_callback,
            this,
            &frame_timer_)) {
        fail_closed("SERVO_FRAME_TIMER_START_FAILED");
        return false;
    }
    frame_timer_started_ = true;
    pio_sm_clear_fifos(pio_, static_cast<uint>(sm_));
    pio_sm_restart(pio_, static_cast<uint>(sm_));
    pulsing_ = true;
    pio_sm_set_enabled(pio_, static_cast<uint>(sm_), true);
    if (!queue_pulse()) {
        fail_closed("SERVO_FIRST_PULSE_QUEUE_FAILED");
        return false;
    }
    last_error_ = "NONE";
    return true;
}

void PioServoOutput::release_resources() {
    if (sm_ >= 0) {
        pio_sm_set_enabled(pio_, static_cast<uint>(sm_), false);
        pio_sm_clear_fifos(pio_, static_cast<uint>(sm_));
        pio_sm_restart(pio_, static_cast<uint>(sm_));
    }
    if (program_offset_ >= 0) {
        pio_remove_program(
            pio_, &servo_pulse_program,
            static_cast<uint>(program_offset_));
        program_offset_ = -1;
    }
    if (sm_ >= 0) {
        pio_sm_unclaim(pio_, static_cast<uint>(sm_));
        sm_ = -1;
    }
}

void PioServoOutput::fail_closed(const char* error) {
    disable();
    last_error_ = error;
}

void PioServoOutput::disable() {
    pulsing_ = false;
    if (frame_timer_started_) {
        cancel_repeating_timer(&frame_timer_);
        frame_timer_started_ = false;
    }
    release_resources();
    enabled_ = false;
    feed_fault_ = false;
    pulse_program_count_ = 0;
    applied_pulse_us_ = 0;
    pio_tick_hz_ = 0.0;
    force_gpio_low();
    last_error_ = "NONE";
}

bool PioServoOutput::is_enabled() const {
    return enabled_;
}

bool PioServoOutput::is_pulsing() const {
    return pulsing_;
}

bool PioServoOutput::has_feed_fault() const {
    return feed_fault_;
}

const char* PioServoOutput::last_error() const {
    return last_error_;
}

uint32_t PioServoOutput::applied_pulse_us() const {
    return applied_pulse_us_;
}

int PioServoOutput::state_machine() const {
    return sm_;
}
