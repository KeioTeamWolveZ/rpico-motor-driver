#include "firmware_logic.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <cstdio>

namespace firmware {
namespace {

const char* skip_spaces(const char* text) {
    while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') {
        ++text;
    }
    return text;
}

bool parse_int_token(const char** cursor, int* value) {
    const char* start = skip_spaces(*cursor);
    if (*start == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    long parsed = std::strtol(start, &end, 10);
    if (end == start || errno == ERANGE ||
        parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    *value = static_cast<int>(parsed);
    *cursor = end;
    return true;
}

bool parse_double_token(const char** cursor, double* value) {
    const char* start = skip_spaces(*cursor);
    if (*start == '\0') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    double parsed = std::strtod(start, &end);
    if (end == start || errno == ERANGE) {
        return false;
    }
    *value = parsed;
    *cursor = end;
    return true;
}

bool gpio_in_list(int gpio, const int* values, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (gpio == values[i]) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool parse_numeric_command(const char* text,
                           NumericCommand* command,
                           NumericParseError* error) {
    if (command == nullptr || error == nullptr || text == nullptr) {
        return false;
    }
    *error = NumericParseError::kNone;
    const char* cursor = skip_spaces(text);
    if (*cursor == '\0') {
        *error = NumericParseError::kEmpty;
        return false;
    }
    if (!parse_int_token(&cursor, &command->id)) {
        *error = NumericParseError::kInvalidId;
        return false;
    }
    if (!parse_int_token(&cursor, &command->mode)) {
        *error = NumericParseError::kInvalidMode;
        return false;
    }
    if (!parse_double_token(&cursor, &command->value)) {
        *error = NumericParseError::kInvalidValue;
        return false;
    }
    if (!std::isfinite(command->value)) {
        *error = NumericParseError::kNonFiniteValue;
        return false;
    }
    if (*skip_spaces(cursor) != '\0') {
        *error = NumericParseError::kTrailingToken;
        return false;
    }
    return true;
}

const char* numeric_parse_error_string(NumericParseError error) {
    switch (error) {
        case NumericParseError::kNone:
            return "NONE";
        case NumericParseError::kEmpty:
            return "EMPTY";
        case NumericParseError::kInvalidId:
            return "INVALID_ID";
        case NumericParseError::kInvalidMode:
            return "INVALID_MODE";
        case NumericParseError::kInvalidValue:
            return "INVALID_VALUE";
        case NumericParseError::kNonFiniteValue:
            return "NON_FINITE_VALUE";
        case NumericParseError::kTrailingToken:
            return "TRAILING_TOKEN";
    }
    return "UNKNOWN";
}

ServoCommandError validate_servo_command(int mode, double command_deg) {
    if (mode != kServoMode) {
        return ServoCommandError::kInvalidMode;
    }
    if (!std::isfinite(command_deg)) {
        return ServoCommandError::kNonFiniteAngle;
    }
    if (command_deg < kServoMinCommandDeg ||
        command_deg > kServoMaxCommandDeg) {
        return ServoCommandError::kAngleOutOfRange;
    }
    return ServoCommandError::kNone;
}

const char* servo_command_error_string(ServoCommandError error) {
    switch (error) {
        case ServoCommandError::kNone:
            return "NONE";
        case ServoCommandError::kInvalidMode:
            return "INVALID_MODE";
        case ServoCommandError::kNonFiniteAngle:
            return "NON_FINITE_ANGLE";
        case ServoCommandError::kAngleOutOfRange:
            return "ANGLE_OUT_OF_RANGE";
    }
    return "UNKNOWN";
}

uint32_t servo_command_to_pulse_us(double command_deg) {
    double fraction =
        (command_deg - kServoMinCommandDeg) /
        (kServoMaxCommandDeg - kServoMinCommandDeg);
    double pulse_us =
        kServoMinPulseUs +
        fraction * static_cast<double>(kServoMaxPulseUs - kServoMinPulseUs);
    return static_cast<uint32_t>(std::lround(pulse_us));
}

bool format_servo_ack(char* buffer,
                      std::size_t buffer_size,
                      int mode,
                      double command_deg,
                      uint32_t applied_pulse_us) {
    if (buffer == nullptr || buffer_size == 0 ||
        !std::isfinite(command_deg)) {
        return false;
    }
    int length = std::snprintf(
        buffer,
        buffer_size,
        "OK servo id=2 mode=%d angle=%.3f pulse_us=%u",
        mode,
        command_deg,
        static_cast<unsigned int>(applied_pulse_us));
    return length >= 0 &&
           static_cast<std::size_t>(length) < buffer_size;
}

ServoLifecycleState servo_lifecycle_after_enable(
    ServoLifecycleState current) {
    return current == ServoLifecycleState::kDisabled
               ? ServoLifecycleState::kEnabledLow
               : current;
}

ServoLifecycleState servo_lifecycle_after_command(
    ServoLifecycleState current,
    bool command_valid) {
    if (!command_valid || current == ServoLifecycleState::kDisabled) {
        return current;
    }
    return ServoLifecycleState::kPulsing;
}

ServoLifecycleState servo_lifecycle_after_disable() {
    return ServoLifecycleState::kDisabled;
}

bool servo_lifecycle_output_is_low(ServoLifecycleState state) {
    return state != ServoLifecycleState::kPulsing;
}

PwmTiming calculate_pwm_timing(uint32_t clock_hz,
                               uint32_t target_frequency_hz) {
    PwmTiming result = {};
    if (clock_hz == 0 || target_frequency_hz == 0 ||
        target_frequency_hz > clock_hz) {
        return result;
    }

    double clocks_per_period =
        static_cast<double>(clock_hz) / target_frequency_hz;
    uint32_t period_counts = 0;
    double divider = 1.0;
    if (clocks_per_period <= 65536.0) {
        period_counts = static_cast<uint32_t>(std::floor(clocks_per_period));
        if (period_counts == 0) {
            return result;
        }
        divider = clocks_per_period / period_counts;
    } else {
        period_counts = 65536;
        divider = clocks_per_period / period_counts;
    }

    double quantized_divider = std::round(divider * 16.0) / 16.0;
    if (quantized_divider < 1.0 || quantized_divider >= 256.0) {
        return result;
    }
    uint32_t divider_scaled =
        static_cast<uint32_t>(std::lround(quantized_divider * 16.0));
    uint32_t divider_integer = divider_scaled / 16;
    uint32_t divider_fraction = divider_scaled % 16;
    if (divider_integer == 0 || divider_integer > 255 ||
        period_counts == 0 || period_counts > 65536) {
        return result;
    }

    result.valid = true;
    result.wrap = static_cast<uint16_t>(period_counts - 1);
    result.divider_integer = static_cast<uint8_t>(divider_integer);
    result.divider_fraction_16 = static_cast<uint8_t>(divider_fraction);
    result.actual_frequency_hz =
        static_cast<double>(clock_hz) /
        (quantized_divider * static_cast<double>(period_counts));
    return result;
}

PioClockTiming calculate_pio_clock_timing(uint32_t clock_hz,
                                         uint32_t target_frequency_hz) {
    PioClockTiming result = {};
    if (clock_hz == 0 || target_frequency_hz == 0) {
        return result;
    }
    double divider = static_cast<double>(clock_hz) / target_frequency_hz;
    if (divider < 1.0 || divider >= 65536.0) {
        return result;
    }
    uint64_t divider_scaled =
        static_cast<uint64_t>(std::llround(divider * 256.0));
    uint64_t divider_integer = divider_scaled / 256;
    uint64_t divider_fraction = divider_scaled % 256;
    if (divider_integer == 0 || divider_integer > 65535) {
        return result;
    }
    double quantized_divider =
        static_cast<double>(divider_scaled) / 256.0;
    result.valid = true;
    result.divider_integer = static_cast<uint16_t>(divider_integer);
    result.divider_fraction_256 = static_cast<uint8_t>(divider_fraction);
    result.actual_frequency_hz =
        static_cast<double>(clock_hz) / quantized_divider;
    return result;
}

uint32_t pio_pulse_program_count(uint32_t pulse_ticks) {
    // The side-set HIGH `mov` and final `jmp x--` consume two HIGH cycles.
    return pulse_ticks >= 2 ? pulse_ticks - 2 : 0;
}

double clamp_double(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

double min_double(double a, double b) {
    return a < b ? a : b;
}

double max_double(double a, double b) {
    return a > b ? a : b;
}

double calculate_sync_tolerance_deg(double target_abs_deg) {
    if (target_abs_deg <= 0.0) {
        return 0.0;
    }
    return min_double(kSyncToleranceDeg,
                      target_abs_deg * kSyncToleranceRatio);
}

double calculate_sync_done_threshold_deg(double target_abs_deg) {
    double tolerance_deg = calculate_sync_tolerance_deg(target_abs_deg);
    return max_double(0.0, target_abs_deg - tolerance_deg);
}

double calculate_sync_min_speed_deg_s(double target_abs_deg) {
    if (target_abs_deg <= kSyncSmallTargetMaxDeg) {
        return kSyncSmallTargetMinSpeedDegS;
    }
    return kSyncMinSpeedDegS;
}

bool sync_start_boost_target_enabled(double target_abs_deg) {
    return target_abs_deg <= kSyncSmallTargetMaxDeg;
}

bool sync_start_boost_wheel_active(bool boost_enabled,
                                   bool boost_active,
                                   int directed_progress_count,
                                   double remaining_deg,
                                   double tolerance_deg,
                                   uint64_t elapsed_us) {
    if (!boost_enabled || !boost_active) {
        return false;
    }
    if (remaining_deg <= tolerance_deg) {
        return false;
    }
    if (directed_progress_count >= kSyncStartBoostProgressCounts) {
        return false;
    }
    if (elapsed_us >= kSyncSmallTargetStartBoostMaxUs) {
        return false;
    }
    return true;
}

double calculate_sync_wheel_base_speed(double remaining_deg,
                                       double requested_speed_deg_s,
                                       double tolerance_deg,
                                       double min_speed_deg_s) {
    if (remaining_deg <= tolerance_deg) {
        return 0.0;
    }
    double base_speed =
        min_double(requested_speed_deg_s, kSyncSlowdownGain * remaining_deg);
    if (base_speed < 0.0) {
        base_speed = 0.0;
    }
    if (base_speed < min_speed_deg_s) {
        base_speed = min_speed_deg_s;
    }
    return clamp_double(base_speed, 0.0, kSyncMaxSpeedDegS);
}

double apply_sync_start_boost(double base_speed_deg_s, bool boost_active) {
    if (boost_active &&
        base_speed_deg_s < kSyncSmallTargetStartBoostSpeedDegS) {
        return kSyncSmallTargetStartBoostSpeedDegS;
    }
    return base_speed_deg_s;
}

SyncSpeedResult calculate_sync_speeds(
    double target,
    double left_progress,
    double right_progress,
    double requested_speed,
    bool left_boost_input,
    bool right_boost_input,
    int left_directed_progress_count,
    int right_directed_progress_count,
    uint64_t elapsed_us) {
    SyncSpeedResult result = {};
    result.tolerance = calculate_sync_tolerance_deg(target);
    result.threshold = calculate_sync_done_threshold_deg(target);
    result.min_speed = calculate_sync_min_speed_deg_s(target);
    result.boost_eligible = sync_start_boost_target_enabled(target);
    result.done = left_progress >= result.threshold &&
                  right_progress >= result.threshold;
    if (result.done) {
        return result;
    }

    double left_remaining = target - left_progress;
    double right_remaining = target - right_progress;
    result.left_boost_active =
        sync_start_boost_wheel_active(
            result.boost_eligible,
            left_boost_input,
            left_directed_progress_count,
            left_remaining,
            result.tolerance,
            elapsed_us);
    result.right_boost_active =
        sync_start_boost_wheel_active(
            result.boost_eligible,
            right_boost_input,
            right_directed_progress_count,
            right_remaining,
            result.tolerance,
            elapsed_us);

    double requested =
        clamp_double(requested_speed, 0.0, kSyncMaxSpeedDegS);
    double left_base =
        calculate_sync_wheel_base_speed(
            left_remaining, requested,
            result.tolerance, result.min_speed);
    double right_base =
        calculate_sync_wheel_base_speed(
            right_remaining, requested,
            result.tolerance, result.min_speed);
    left_base =
        apply_sync_start_boost(left_base, result.left_boost_active);
    right_base =
        apply_sync_start_boost(right_base, result.right_boost_active);
    double correction = kSyncKp * (left_progress - right_progress);

    result.left_abs =
        clamp_double(left_base - correction, 0.0, kSyncMaxSpeedDegS);
    result.right_abs =
        clamp_double(right_base + correction, 0.0, kSyncMaxSpeedDegS);
    if (left_remaining <= result.tolerance) {
        result.left_abs = 0.0;
    }
    if (right_remaining <= result.tolerance) {
        result.right_abs = 0.0;
    }
    return result;
}

uint64_t calculate_sync_total_timeout_us(double target_abs_deg,
                                         double requested_speed_deg_s) {
    if (!std::isfinite(target_abs_deg) ||
        !std::isfinite(requested_speed_deg_s) ||
        target_abs_deg <= 0.0 || requested_speed_deg_s <= 0.0) {
        return kSyncMinimumTotalTimeoutUs;
    }
    double effective_speed =
        clamp_double(requested_speed_deg_s,
                     kSyncMinSpeedDegS,
                     kSyncMaxSpeedDegS);
    double expected_us =
        target_abs_deg / effective_speed * 1000000.0;
    double timeout_us =
        expected_us * kSyncTotalTimeoutMultiplier +
        static_cast<double>(kSyncTotalTimeoutMarginUs);
    timeout_us =
        clamp_double(timeout_us,
                     static_cast<double>(kSyncMinimumTotalTimeoutUs),
                     static_cast<double>(kSyncMaximumTotalTimeoutUs));
    return static_cast<uint64_t>(std::llround(timeout_us));
}

bool deadline_expired(uint64_t now_us,
                      uint64_t start_us,
                      uint64_t timeout_us) {
    return timeout_us > 0 && now_us - start_us >= timeout_us;
}

bool is_diagnostic_protected_gpio(int gpio,
                                  int uart_tx_gpio,
                                  int uart_rx_gpio,
                                  int servo_gpio,
                                  const int* motor_gpios,
                                  std::size_t motor_gpio_count,
                                  const int* encoder_gpios,
                                  std::size_t encoder_gpio_count,
                                  int led_gpio) {
    return gpio == uart_tx_gpio || gpio == uart_rx_gpio ||
           gpio == servo_gpio || gpio == led_gpio ||
           gpio_in_list(gpio, motor_gpios, motor_gpio_count) ||
           gpio_in_list(gpio, encoder_gpios, encoder_gpio_count);
}

bool invoke_safe_stop(const SafeStopCallbacks& callbacks) {
    if (callbacks.stop_motors == nullptr ||
        callbacks.stop_servo == nullptr ||
        callbacks.stop_diagnostics == nullptr) {
        return false;
    }
    callbacks.stop_motors();
    callbacks.stop_servo();
    callbacks.stop_diagnostics();
    return true;
}

bool timer_registration_succeeded(bool velocity_timer_ok,
                                  bool position_timer_ok) {
    return velocity_timer_ok && position_timer_ok;
}

}  // namespace firmware
