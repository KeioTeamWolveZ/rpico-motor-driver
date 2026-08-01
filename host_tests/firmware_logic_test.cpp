#include "firmware_logic.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <cstring>

namespace {

int failures = 0;
int stop_motor_calls = 0;
int stop_servo_calls = 0;
int stop_diag_calls = 0;

void expect_true(const char* name, bool value) {
    if (!value) {
        std::printf("FAIL %s\n", name);
        ++failures;
    }
}

void expect_false(const char* name, bool value) {
    expect_true(name, !value);
}

void expect_equal(const char* name, long long actual, long long expected) {
    if (actual != expected) {
        std::printf("FAIL %s actual=%lld expected=%lld\n",
                    name, actual, expected);
        ++failures;
    }
}

void expect_close(const char* name, double actual, double expected,
                  double tolerance = 0.001) {
    if (std::fabs(actual - expected) > tolerance) {
        std::printf("FAIL %s actual=%.9f expected=%.9f\n",
                    name, actual, expected);
        ++failures;
    }
}

void expect_contains(const char* name, const char* text, const char* pattern) {
    if (std::strstr(text, pattern) == nullptr) {
        std::printf("FAIL %s missing=%s text=%s\n", name, pattern, text);
        ++failures;
    }
}

void expect_not_contains(const char* name,
                         const char* text,
                         const char* pattern) {
    if (std::strstr(text, pattern) != nullptr) {
        std::printf("FAIL %s unexpected=%s text=%s\n", name, pattern, text);
        ++failures;
    }
}

void stop_motors() {
    ++stop_motor_calls;
}

void stop_servo() {
    ++stop_servo_calls;
}

void stop_diagnostics() {
    ++stop_diag_calls;
}

void test_numeric_parser() {
    firmware::NumericCommand command = {};
    firmware::NumericParseError error =
        firmware::NumericParseError::kNone;
    expect_true("parse servo mode 0",
                firmware::parse_numeric_command(
                    "2 0 90", &command, &error));
    expect_equal("parsed id", command.id, 2);
    expect_equal("parsed mode", command.mode, 0);
    expect_close("parsed value", command.value, 90.0);

    expect_true("legacy wheel velocity command accepted",
                firmware::parse_numeric_command(
                    "0 0 -120", &command, &error));
    expect_equal("wheel velocity id", command.id, 0);
    expect_equal("wheel velocity mode", command.mode, 0);
    expect_close("wheel velocity value", command.value, -120.0);
    expect_true("legacy wheel position command accepted",
                firmware::parse_numeric_command(
                    "1 1 30", &command, &error));
    expect_equal("wheel position id", command.id, 1);
    expect_equal("wheel position mode", command.mode, 1);
    expect_close("wheel position value", command.value, 30.0);

    expect_false("reject incomplete",
                 firmware::parse_numeric_command(
                     "2 0", &command, &error));
    expect_false("reject trailing token",
                 firmware::parse_numeric_command(
                     "2 0 90 extra", &command, &error));
    expect_true("trailing error",
                error == firmware::NumericParseError::kTrailingToken);
    expect_false("reject nan",
                 firmware::parse_numeric_command(
                     "2 0 nan", &command, &error));
    expect_true("nan error",
                error == firmware::NumericParseError::kNonFiniteValue);
    expect_false("reject inf",
                 firmware::parse_numeric_command(
                     "2 0 inf", &command, &error));
    expect_false("reject negative inf",
                 firmware::parse_numeric_command(
                     "2 0 -inf", &command, &error));
}

void test_servo_validation_and_mapping() {
    expect_true("mode 0 accepted",
                firmware::validate_servo_command(0, 90.0) ==
                    firmware::ServoCommandError::kNone);
    expect_true("invalid mode rejected",
                firmware::validate_servo_command(1, 90.0) ==
                    firmware::ServoCommandError::kInvalidMode);
    expect_true("-1 rejected",
                firmware::validate_servo_command(0, -1.0) ==
                    firmware::ServoCommandError::kAngleOutOfRange);
    expect_true("0 accepted",
                firmware::validate_servo_command(0, 0.0) ==
                    firmware::ServoCommandError::kNone);
    expect_true("180 accepted",
                firmware::validate_servo_command(0, 180.0) ==
                    firmware::ServoCommandError::kNone);
    expect_true("181 rejected",
                firmware::validate_servo_command(0, 181.0) ==
                    firmware::ServoCommandError::kAngleOutOfRange);
    expect_true("nan rejected",
                firmware::validate_servo_command(
                    0, std::numeric_limits<double>::quiet_NaN()) ==
                    firmware::ServoCommandError::kNonFiniteAngle);
    expect_true("inf rejected",
                firmware::validate_servo_command(
                    0, std::numeric_limits<double>::infinity()) ==
                    firmware::ServoCommandError::kNonFiniteAngle);
    expect_equal("0 maps to 500 us",
                 firmware::servo_command_to_pulse_us(0.0), 500);
    expect_equal("90 maps to provisional neutral 1450 us",
                 firmware::servo_command_to_pulse_us(90.0), 1450);
    expect_equal("180 maps to 2400 us",
                 firmware::servo_command_to_pulse_us(180.0), 2400);
    char acknowledgement[96] = {};
    expect_true("servo ACK formats",
                firmware::format_servo_ack(
                    acknowledgement,
                    sizeof(acknowledgement),
                    0,
                    90.0,
                    firmware::servo_command_to_pulse_us(90.0)));
    expect_true("servo ACK matches applied command",
                std::strcmp(
                    acknowledgement,
                    "OK servo id=2 mode=0 angle=90.000 pulse_us=1450") == 0);
    char short_acknowledgement[8] = {};
    expect_false("truncated servo ACK rejected",
                 firmware::format_servo_ack(
                     short_acknowledgement,
                     sizeof(short_acknowledgement),
                     0,
                     90.0,
                     1450));

    firmware::ServoLifecycleState state =
        firmware::ServoLifecycleState::kDisabled;
    expect_true("disabled output LOW",
                firmware::servo_lifecycle_output_is_low(state));
    state = firmware::servo_lifecycle_after_enable(state);
    expect_true("enable keeps output LOW",
                firmware::servo_lifecycle_output_is_low(state));
    state = firmware::servo_lifecycle_after_command(state, true);
    expect_true("valid command starts pulses",
                state == firmware::ServoLifecycleState::kPulsing);
    state = firmware::servo_lifecycle_after_disable();
    expect_true("disable returns output LOW",
                firmware::servo_lifecycle_output_is_low(state));
}

void test_clock_calculation() {
    expect_false("servo does not use hardware PWM",
                 firmware::kServoUsesHardwarePwm);
    expect_equal("GPIO5 hardware PWM slice",
                 firmware::rp2_pwm_slice_for_gpio(5), 2);
    expect_equal("GPIO20 hardware PWM slice",
                 firmware::rp2_pwm_slice_for_gpio(20), 2);
    expect_equal("GPIO21 hardware PWM slice",
                 firmware::rp2_pwm_slice_for_gpio(21), 2);

    firmware::PwmTiming pwm125 =
        firmware::calculate_pwm_timing(125000000, 50000);
    expect_true("125 MHz PWM valid", pwm125.valid);
    expect_equal("125 MHz PWM wrap", pwm125.wrap, 2499);
    expect_equal("125 MHz PWM divider", pwm125.divider_integer, 1);
    expect_close("125 MHz PWM frequency",
                 pwm125.actual_frequency_hz, 50000.0);

    firmware::PwmTiming pwm150 =
        firmware::calculate_pwm_timing(150000000, 50000);
    expect_true("150 MHz PWM valid", pwm150.valid);
    expect_equal("150 MHz PWM wrap", pwm150.wrap, 2999);
    expect_equal("150 MHz PWM divider", pwm150.divider_integer, 1);
    expect_close("150 MHz PWM frequency",
                 pwm150.actual_frequency_hz, 50000.0);

    firmware::PioClockTiming pio125 =
        firmware::calculate_pio_clock_timing(125000000, 1000000);
    firmware::PioClockTiming pio150 =
        firmware::calculate_pio_clock_timing(150000000, 1000000);
    expect_true("125 MHz PIO valid", pio125.valid);
    expect_true("150 MHz PIO valid", pio150.valid);
    expect_close("125 MHz PIO 1 MHz",
                 pio125.actual_frequency_hz, 1000000.0);
    expect_close("150 MHz PIO 1 MHz",
                 pio150.actual_frequency_hz, 1000000.0);
    expect_equal("500 us encoded count",
                 firmware::pio_pulse_program_count(500), 498);
    expect_equal("2400 us encoded count",
                 firmware::pio_pulse_program_count(2400), 2398);
}

void test_timeout_and_safe_stop() {
    expect_false("partial line before deadline",
                 firmware::deadline_expired(
                     499999, 0,
                     firmware::kUartPartialLineTimeoutUs));
    expect_true("partial line at deadline",
                firmware::deadline_expired(
                    500000, 0,
                    firmware::kUartPartialLineTimeoutUs));
    expect_true("watchdog timeout",
                firmware::deadline_expired(
                    firmware::kCommandWatchdogTimeoutUs,
                    0,
                    firmware::kCommandWatchdogTimeoutUs));
    expect_true("sync stall timeout",
                firmware::deadline_expired(
                    firmware::kSyncStallTimeoutUs,
                    0,
                    firmware::kSyncStallTimeoutUs));
    expect_equal("short sync uses minimum total timeout",
                 firmware::calculate_sync_total_timeout_us(5.0, 180.0),
                 firmware::kSyncMinimumTotalTimeoutUs);
    expect_equal("long sync total timeout is bounded",
                 firmware::calculate_sync_total_timeout_us(100000.0, 1.0),
                 firmware::kSyncMaximumTotalTimeoutUs);

    firmware::SafeStopCallbacks callbacks = {
        stop_motors, stop_servo, stop_diagnostics};
    expect_true("safe stop callbacks",
                firmware::invoke_safe_stop(callbacks));
    expect_equal("motor stop called", stop_motor_calls, 1);
    expect_equal("servo stop called", stop_servo_calls, 1);
    expect_equal("diag stop called", stop_diag_calls, 1);

    expect_true("both timers accepted",
                firmware::timer_registration_succeeded(true, true));
    expect_false("velocity timer failure",
                 firmware::timer_registration_succeeded(false, true));
    expect_false("position timer failure",
                 firmware::timer_registration_succeeded(true, false));
}

void test_diagnostic_protection() {
    const int motor_gpios[] = {20, 21, 12, 11};
    const int encoder_gpios[] = {6, 7, 3, 4};
    for (int gpio : motor_gpios) {
        expect_true("motor GPIO protected",
                    firmware::is_diagnostic_protected_gpio(
                        gpio, 0, 1, 5,
                        motor_gpios, 4, encoder_gpios, 4, 25));
    }
    for (int gpio : encoder_gpios) {
        expect_true("encoder GPIO protected",
                    firmware::is_diagnostic_protected_gpio(
                        gpio, 0, 1, 5,
                        motor_gpios, 4, encoder_gpios, 4, 25));
    }
    expect_true("servo GPIO protected",
                firmware::is_diagnostic_protected_gpio(
                    5, 0, 1, 5,
                    motor_gpios, 4, encoder_gpios, 4, 25));
    expect_true("UART GPIO protected",
                firmware::is_diagnostic_protected_gpio(
                    0, 0, 1, 5,
                    motor_gpios, 4, encoder_gpios, 4, 25));
    expect_false("unrelated GPIO available",
                 firmware::is_diagnostic_protected_gpio(
                     8, 0, 1, 5,
                     motor_gpios, 4, encoder_gpios, 4, 25));
}

firmware::SyncFaultSnapshot sample_sync_fault(
    firmware::SyncFaultType type) {
    firmware::SyncFaultSnapshot snapshot = {};
    snapshot.valid = true;
    snapshot.type = type;
    snapshot.wheel_id =
        type == firmware::SyncFaultType::kLeftStall ? 1 : 0;
    snapshot.target_deg = 35.569;
    snapshot.requested_speed_deg_s = 180.0;
    snapshot.command_start_us = 1000000;
    snapshot.fault_us = 2500000;
    snapshot.elapsed_ms = 1500;
    snapshot.left_raw_count = 123;
    snapshot.right_raw_count = 456;
    snapshot.left_prev_raw_count = 122;
    snapshot.right_prev_raw_count = 456;
    snapshot.left_delta_count = 3;
    snapshot.right_delta_count = 0;
    snapshot.left_progress_deg = 0.277;
    snapshot.right_progress_deg = 0.0;
    snapshot.left_remaining_deg = 35.292;
    snapshot.right_remaining_deg = 35.569;
    snapshot.left_speed_deg_s = 42.0;
    snapshot.right_speed_deg_s = 43.0;
    snapshot.correction_deg_s = 0.083;
    snapshot.left_reached = false;
    snapshot.right_reached = false;
    snapshot.left_dir_sign = -1;
    snapshot.right_dir_sign = -1;
    snapshot.left_idle_ms = 10;
    snapshot.right_idle_ms = 1500;
    snapshot.left_last_progress_us = 2490000;
    snapshot.right_last_progress_us = 1000000;
    return snapshot;
}

void test_sync_fault_status_format() {
    char line[1200] = {};
    firmware::SyncFaultSnapshot none = {};
    expect_true("none fault formats",
                firmware::format_sync_fault_status(
                    line, sizeof(line), none));
    expect_true("none fault exact",
                std::strcmp(line, "SYNC_FAULT state=NONE") == 0);

    firmware::SyncFaultSnapshot left =
        sample_sync_fault(firmware::SyncFaultType::kLeftStall);
    expect_true("left fault formats",
                firmware::format_sync_fault_status(
                    line, sizeof(line), left));
    expect_contains("left type", line, "type=LEFT_STALL");
    expect_contains("left wheel id", line, "wheel_id=1");
    expect_contains("left raw mapping", line, "left_raw_count=123");
    expect_contains("right raw mapping", line, "right_raw_count=456");
    expect_contains("left idle", line, "left_idle_ms=10");
    expect_contains("right idle", line, "right_idle_ms=1500");
    expect_contains("line prefix", line, "SYNC_FAULT state=VALID");
    expect_not_contains("no newline", line, "\n");
    expect_not_contains("no nan", line, "nan");
    expect_not_contains("no inf", line, "inf");

    firmware::SyncFaultSnapshot right =
        sample_sync_fault(firmware::SyncFaultType::kRightStall);
    expect_true("right fault formats",
                firmware::format_sync_fault_status(
                    line, sizeof(line), right));
    expect_contains("right type", line, "type=RIGHT_STALL");
    expect_contains("right wheel id", line, "wheel_id=0");
    expect_contains("right is enc0 value", line, "right_raw_count=456");

    firmware::SyncFaultSnapshot cleared = {};
    expect_true("clear returns none",
                firmware::format_sync_fault_status(
                    line, sizeof(line), cleared));
    expect_true("clear exact",
                std::strcmp(line, "SYNC_FAULT state=NONE") == 0);

    firmware::SyncFaultSnapshot bad = right;
    bad.left_progress_deg = std::numeric_limits<double>::infinity();
    expect_false("non-finite fault rejected",
                 firmware::format_sync_fault_status(
                     line, sizeof(line), bad));
}

}  // namespace

int main() {
    test_numeric_parser();
    test_servo_validation_and_mapping();
    test_clock_calculation();
    test_timeout_and_safe_stop();
    test_diagnostic_protection();
    test_sync_fault_status_format();
    if (failures != 0) {
        std::printf("firmware_logic_test FAIL count=%d\n", failures);
        return 1;
    }
    std::printf("firmware_logic_test OK\n");
    return 0;
}
