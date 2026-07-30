#pragma once

#include <cstddef>
#include <cstdint>

namespace firmware {

constexpr double kSyncKp = 0.30;
constexpr double kSyncToleranceDeg = 3.0;
constexpr double kSyncToleranceRatio = 0.20;
constexpr double kSyncMinSpeedDegS = 20.0;
constexpr double kSyncSmallTargetMinSpeedDegS = 30.0;
constexpr double kSyncSmallTargetMaxDeg = 15.0;
constexpr double kSyncSmallTargetStartBoostSpeedDegS = 80.0;
constexpr uint64_t kSyncSmallTargetStartBoostMaxUs = 300000;
constexpr int kSyncStartBoostProgressCounts = 1;
constexpr double kSyncMaxSpeedDegS = 180.0;
constexpr double kSyncSlowdownGain = 1.2;

// These are provisional safety limits. Normal UART commands at 115200 baud
// complete in milliseconds, while the existing Pi servo trial waits about
// 0.6 seconds between commands. Calibration may justify different values.
constexpr uint64_t kUartPartialLineTimeoutUs = 500000;
constexpr uint64_t kCommandWatchdogTimeoutUs = 2000000;
constexpr uint64_t kSyncStallTimeoutUs = 1500000;
constexpr uint64_t kSyncMinimumTotalTimeoutUs = 5000000;
constexpr uint64_t kSyncMaximumTotalTimeoutUs = 60000000;
constexpr uint64_t kSyncTotalTimeoutMarginUs = 2000000;
constexpr double kSyncTotalTimeoutMultiplier = 3.0;

// Continuous-rotation servos interpret this "angle" as a pulse-width command,
// not as a physical shaft angle. Neutral=90 is provisional until each servo is
// calibrated. Preserve the legacy 0.5 ms to 2.4 ms range.
constexpr int kServoMode = 0;
constexpr double kServoMinCommandDeg = 0.0;
constexpr double kServoNeutralCommandDeg = 90.0;
constexpr double kServoMaxCommandDeg = 180.0;
constexpr uint32_t kServoMinPulseUs = 500;
constexpr uint32_t kServoMaxPulseUs = 2400;
constexpr uint32_t kServoFrameUs = 20000;
constexpr uint32_t kServoPioTickHz = 1000000;
constexpr bool kServoUsesHardwarePwm = false;

constexpr int rp2_pwm_slice_for_gpio(int gpio) {
    return gpio < 32 ? ((gpio >> 1) & 7)
                     : (8 + ((gpio >> 1) & 3));
}

struct NumericCommand {
    int id;
    int mode;
    double value;
};

enum class NumericParseError {
    kNone,
    kEmpty,
    kInvalidId,
    kInvalidMode,
    kInvalidValue,
    kNonFiniteValue,
    kTrailingToken,
};

bool parse_numeric_command(const char* text,
                           NumericCommand* command,
                           NumericParseError* error);
const char* numeric_parse_error_string(NumericParseError error);

enum class ServoCommandError {
    kNone,
    kInvalidMode,
    kNonFiniteAngle,
    kAngleOutOfRange,
};

ServoCommandError validate_servo_command(int mode, double command_deg);
const char* servo_command_error_string(ServoCommandError error);
uint32_t servo_command_to_pulse_us(double command_deg);
bool format_servo_ack(char* buffer,
                      std::size_t buffer_size,
                      int mode,
                      double command_deg,
                      uint32_t applied_pulse_us);

enum class ServoLifecycleState {
    kDisabled,
    kEnabledLow,
    kPulsing,
};

ServoLifecycleState servo_lifecycle_after_enable(
    ServoLifecycleState current);
ServoLifecycleState servo_lifecycle_after_command(
    ServoLifecycleState current,
    bool command_valid);
ServoLifecycleState servo_lifecycle_after_disable();
bool servo_lifecycle_output_is_low(ServoLifecycleState state);

struct PwmTiming {
    bool valid;
    uint16_t wrap;
    uint8_t divider_integer;
    uint8_t divider_fraction_16;
    double actual_frequency_hz;
};

PwmTiming calculate_pwm_timing(uint32_t clock_hz, uint32_t target_frequency_hz);

struct PioClockTiming {
    bool valid;
    uint16_t divider_integer;
    uint8_t divider_fraction_256;
    double actual_frequency_hz;
};

PioClockTiming calculate_pio_clock_timing(uint32_t clock_hz,
                                         uint32_t target_frequency_hz);
uint32_t pio_pulse_program_count(uint32_t pulse_ticks);

double clamp_double(double value, double min_value, double max_value);
double min_double(double a, double b);
double max_double(double a, double b);
double calculate_sync_tolerance_deg(double target_abs_deg);
double calculate_sync_done_threshold_deg(double target_abs_deg);
double calculate_sync_min_speed_deg_s(double target_abs_deg);
bool sync_start_boost_target_enabled(double target_abs_deg);
bool sync_start_boost_wheel_active(bool boost_enabled,
                                   bool boost_active,
                                   int directed_progress_count,
                                   double remaining_deg,
                                   double tolerance_deg,
                                   uint64_t elapsed_us);
double calculate_sync_wheel_base_speed(double remaining_deg,
                                       double requested_speed_deg_s,
                                       double tolerance_deg,
                                       double min_speed_deg_s);
double apply_sync_start_boost(double base_speed_deg_s, bool boost_active);

struct SyncSpeedResult {
    double left_abs;
    double right_abs;
    bool done;
    double tolerance;
    double threshold;
    double min_speed;
    bool boost_eligible;
    bool left_boost_active;
    bool right_boost_active;
};

SyncSpeedResult calculate_sync_speeds(
    double target,
    double left_progress,
    double right_progress,
    double requested_speed,
    bool left_boost_input = false,
    bool right_boost_input = false,
    int left_directed_progress_count = 0,
    int right_directed_progress_count = 0,
    uint64_t elapsed_us = 0);

uint64_t calculate_sync_total_timeout_us(double target_abs_deg,
                                         double requested_speed_deg_s);
bool deadline_expired(uint64_t now_us, uint64_t start_us, uint64_t timeout_us);

bool is_diagnostic_protected_gpio(int gpio,
                                  int uart_tx_gpio,
                                  int uart_rx_gpio,
                                  int servo_gpio,
                                  const int* motor_gpios,
                                  std::size_t motor_gpio_count,
                                  const int* encoder_gpios,
                                  std::size_t encoder_gpio_count,
                                  int led_gpio);

using StopCallback = void (*)();

struct SafeStopCallbacks {
    StopCallback stop_motors;
    StopCallback stop_servo;
    StopCallback stop_diagnostics;
};

bool invoke_safe_stop(const SafeStopCallbacks& callbacks);
bool timer_registration_succeeded(bool velocity_timer_ok,
                                  bool position_timer_ok);

}  // namespace firmware
