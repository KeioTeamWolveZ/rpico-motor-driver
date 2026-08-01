#include <ctype.h>
#include <cmath>
#include <new>
#include <stdio.h>
#include <string.h>

#include "firmware_logic.h"
#include "lib/rpico-encoder-plus/qenc.h"
#include "lib/rpico-motor/motor.h"
#include "lib/rpico-pwm/pwm.h"
#include "lib/rpico-servo/servo.h"
#include "pio_servo_output.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"

#ifndef MOTOR_UART_ID
#define MOTOR_UART_ID 0
#endif

#ifndef MOTOR_FIRMWARE_VERSION
#define MOTOR_FIRMWARE_VERSION "dev"
#endif

#ifndef MOTOR_PICO_BOARD
#define MOTOR_PICO_BOARD "unknown"
#endif

#ifndef MOTOR_UART_BAUDRATE
#define MOTOR_UART_BAUDRATE 115200
#endif

#ifndef MOTOR_UART_TX_GPIO
#define MOTOR_UART_TX_GPIO 0
#endif

#ifndef MOTOR_UART_RX_GPIO
#define MOTOR_UART_RX_GPIO 1
#endif

#ifndef MOTOR_SERVO_GPIO
#define MOTOR_SERVO_GPIO 5
#endif

#ifndef MOTOR_PWM0_GPIO
#define MOTOR_PWM0_GPIO 20
#endif

#ifndef MOTOR_PWM1_GPIO
#define MOTOR_PWM1_GPIO 21
#endif

#ifndef MOTOR_PWM2_GPIO
#define MOTOR_PWM2_GPIO 12
#endif

#ifndef MOTOR_PWM3_GPIO
#define MOTOR_PWM3_GPIO 11
#endif

#ifndef MOTOR_QENC0_GPIO
#define MOTOR_QENC0_GPIO 6
#endif

#ifndef MOTOR_QENC1_GPIO
#define MOTOR_QENC1_GPIO 3
#endif

#if MOTOR_UART_ID == 0
#define MOTOR_UART_INSTANCE uart0
#define MOTOR_UART_INSTANCE_NAME "uart0"
#elif MOTOR_UART_ID == 1
#define MOTOR_UART_INSTANCE uart1
#define MOTOR_UART_INSTANCE_NAME "uart1"
#else
#error "MOTOR_UART_ID must be 0 or 1"
#endif

#ifdef PICO_DEFAULT_LED_PIN
static const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#else
static const uint LED_PIN = 25;
#endif

static const bool LED_ON = false;
static const bool LED_OFF = true;
static const uint32_t MORSE_DOT_MS = 150;
static const uint32_t MORSE_DASH_MS = 500;
static const uint32_t MORSE_SYMBOL_GAP_MS = 150;
static const uint32_t MORSE_CHAR_GAP_MS = 1000;
static const uint32_t LOOP_ALIVE_SERVICE_US = 10000;
static const int DIAG_MAX_GPIO = 29;
static const int DIAG_PWM_FREQ = 50000;
static const double ENCODER_COUNTS_PER_REV = 3900.0;

enum SyncError {
    SYNC_ERROR_NONE = 0,
    SYNC_ERROR_TOTAL_TIMEOUT,
    SYNC_ERROR_LEFT_STALL,
    SYNC_ERROR_RIGHT_STALL,
};

struct SyncRotationState {
    bool active;
    bool done;
    bool ever_started;
    int left_start_count;
    int right_start_count;
    double target_abs_deg;
    int left_dir_sign;
    int right_dir_sign;
    double base_speed_deg_s;
    double last_left_progress_deg;
    double last_right_progress_deg;
    double last_error_deg;
    double last_left_speed_cmd_deg_s;
    double last_right_speed_cmd_deg_s;
    double final_left_progress_deg;
    double final_right_progress_deg;
    double final_error_deg;
    bool start_boost_enabled;
    bool left_start_boost_active;
    bool right_start_boost_active;
    uint64_t start_boost_start_us;
    uint64_t operation_start_us;
    uint64_t total_timeout_us;
    uint64_t left_last_progress_us;
    uint64_t right_last_progress_us;
    int left_last_raw_count;
    int right_last_raw_count;
    SyncError error;
    bool error_report_pending;
};

PioServoOutput servo_output(MOTOR_SERVO_GPIO);
Qenc enc[2] = {Qenc(MOTOR_QENC0_GPIO), Qenc(MOTOR_QENC1_GPIO)};
static Pwm* pwm0 = NULL;
static Pwm* pwm1 = NULL;
static Pwm* pwm2 = NULL;
static Pwm* pwm3 = NULL;
static Motor* motor0 = NULL;
static Motor* motor1 = NULL;

static char buf[255];
static char current_led_char = 'L';
static const char* current_led_pattern = ".-..";
static bool led_active = false;
static bool led_symbol_on = false;
static int led_pattern_index = 0;
static uint64_t led_next_event_us = 0;

static volatile bool motors_runtime_enabled = false;
static volatile bool motors_initialized = false;
static bool servo_runtime_enabled = false;
static volatile bool timer_started = false;
static volatile bool sync_start_initializing = false;
static volatile SyncRotationState sync_state = {};
static firmware::SyncFaultSnapshot sync_last_fault = {};
static repeating_timer_t velocity_timer;
static repeating_timer_t position_timer;
static bool diag_high[DIAG_MAX_GPIO + 1] = {false};
static bool diag_pwm[DIAG_MAX_GPIO + 1] = {false};
static volatile bool motor_motion_command_active[2] = {false, false};
static uint64_t last_complete_command_us = 0;
static bool watchdog_report_pending = false;
static const char* motor_enable_error = "NONE";

static bool set_led_char(char led_char);
static void safe_all();
static void service_runtime_safety();

static void force_motor_gpio_low(uint gpio) {
    gpio_init(gpio);
    gpio_set_function(gpio, GPIO_FUNC_SIO);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
}

static void force_motor_outputs_low() {
    force_motor_gpio_low(MOTOR_PWM0_GPIO);
    force_motor_gpio_low(MOTOR_PWM1_GPIO);
    force_motor_gpio_low(MOTOR_PWM2_GPIO);
    force_motor_gpio_low(MOTOR_PWM3_GPIO);
}

static bool restore_motor_pwm_outputs() {
    if (pwm0 == NULL || pwm1 == NULL || pwm2 == NULL || pwm3 == NULL) {
        force_motor_outputs_low();
        return false;
    }
    bool restored = pwm0->restore() && pwm1->restore() &&
                    pwm2->restore() && pwm3->restore();
    if (!restored) {
        force_motor_outputs_low();
    }
    return restored;
}

static bool create_motor_objects_if_needed() {
    if (pwm0 != NULL && pwm1 != NULL && pwm2 != NULL && pwm3 != NULL &&
        motor0 != NULL && motor1 != NULL) {
        return true;
    }

    if (pwm0 == NULL) {
        pwm0 = new (std::nothrow) Pwm(MOTOR_PWM0_GPIO, 50000);
    }
    if (pwm1 == NULL) {
        pwm1 = new (std::nothrow) Pwm(MOTOR_PWM1_GPIO, 50000);
    }
    if (pwm2 == NULL) {
        pwm2 = new (std::nothrow) Pwm(MOTOR_PWM2_GPIO, 50000);
    }
    if (pwm3 == NULL) {
        pwm3 = new (std::nothrow) Pwm(MOTOR_PWM3_GPIO, 50000);
    }

    if (pwm0 == NULL || pwm1 == NULL || pwm2 == NULL || pwm3 == NULL) {
        force_motor_outputs_low();
        return false;
    }

    if (motor0 == NULL) {
        motor0 = new (std::nothrow) Motor(*pwm3, *pwm0, enc[0]);
    }
    if (motor1 == NULL) {
        motor1 = new (std::nothrow) Motor(*pwm1, *pwm2, enc[1]);
    }

    force_motor_outputs_low();
    return motor0 != NULL && motor1 != NULL;
}

static bool pin_is_uart(int pin) {
    return pin == MOTOR_UART_TX_GPIO || pin == MOTOR_UART_RX_GPIO;
}

static bool motors_pin_conflict() {
    return pin_is_uart(MOTOR_PWM0_GPIO) || pin_is_uart(MOTOR_PWM1_GPIO) ||
           pin_is_uart(MOTOR_PWM2_GPIO) || pin_is_uart(MOTOR_PWM3_GPIO) ||
           pin_is_uart(MOTOR_QENC0_GPIO) || pin_is_uart(MOTOR_QENC0_GPIO + 1) ||
           pin_is_uart(MOTOR_QENC1_GPIO) || pin_is_uart(MOTOR_QENC1_GPIO + 1);
}

static bool servo_pin_conflict() {
    return pin_is_uart(MOTOR_SERVO_GPIO);
}

static bool diag_gpio_valid(int gpio) {
    return gpio >= 0 && gpio <= DIAG_MAX_GPIO;
}

static bool diag_gpio_protected(int gpio) {
    const int motor_gpios[] = {
        MOTOR_PWM0_GPIO, MOTOR_PWM1_GPIO,
        MOTOR_PWM2_GPIO, MOTOR_PWM3_GPIO};
    const int encoder_gpios[] = {
        MOTOR_QENC0_GPIO, MOTOR_QENC0_GPIO + 1,
        MOTOR_QENC1_GPIO, MOTOR_QENC1_GPIO + 1};
    return firmware::is_diagnostic_protected_gpio(
        gpio,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO,
        MOTOR_SERVO_GPIO,
        motor_gpios,
        sizeof(motor_gpios) / sizeof(motor_gpios[0]),
        encoder_gpios,
        sizeof(encoder_gpios) / sizeof(encoder_gpios[0]),
        LED_PIN) ||
        gpio == 22 || gpio == 23;
}

static bool diag_runtime_available() {
    if (motors_runtime_enabled) {
        printf("ERR MOTORS_ENABLED\n");
        fflush(stdout);
        set_led_char('E');
        return false;
    }
    if (servo_runtime_enabled) {
        printf("ERR SERVO_ENABLED\n");
        fflush(stdout);
        set_led_char('E');
        return false;
    }
    return true;
}

static bool diag_validate_gpio(int gpio) {
    if (!diag_gpio_valid(gpio)) {
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return false;
    }
    if (diag_gpio_protected(gpio)) {
        printf("ERR PROTECTED_PIN\n");
        fflush(stdout);
        set_led_char('E');
        return false;
    }
    return true;
}

static void print_pin_conflict_value() {
    bool any = false;
    if (motors_pin_conflict()) {
        printf("motors");
        any = true;
    }
    if (servo_pin_conflict()) {
        printf("%sservo", any ? "," : "");
        any = true;
    }
    if (!any) {
        printf("none");
    }
}

static void led_write(bool on) {
    gpio_put(LED_PIN, on ? LED_ON : LED_OFF);
}

static void init_led() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led_write(false);
}

static const char* morse_pattern_for(char led_char) {
    switch (led_char) {
        case 'B':
            return "-...";
        case 'U':
            return "..-";
        case 'O':
            return "---";
        case 'A':
            return ".-";
        case 'N':
            return "-.";
        case 'M':
            return "--";
        case 'S':
            return "...";
        case 'L':
            return ".-..";
        case 'R':
            return ".-.";
        case 'P':
            return ".--.";
        case 'E':
            return ".";
        case 'I':
            return "..";
        case 'T':
            return "-";
        case 'X':
            return "-..-";
        default:
            return NULL;
    }
}

static bool set_led_char(char led_char) {
    led_char = (char)toupper((unsigned char)led_char);
    const char* pattern = morse_pattern_for(led_char);
    if (pattern == NULL) {
        return false;
    }

    current_led_char = led_char;
    current_led_pattern = pattern;
    led_active = false;
    led_symbol_on = false;
    led_pattern_index = 0;
    led_next_event_us = 0;
    led_write(false);
    return true;
}

static void init_uart_stdio() {
    stdio_uart_init_full(
        MOTOR_UART_INSTANCE,
        MOTOR_UART_BAUDRATE,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO);
}

static void log_uart_config() {
    printf("DBG firmware=%s\n", MOTOR_FIRMWARE_VERSION);
    printf("DBG build_date=%s %s\n", __DATE__, __TIME__);
    printf("DBG board=%s\n", MOTOR_PICO_BOARD);
    printf("DBG uart_instance=%s\n", MOTOR_UART_INSTANCE_NAME);
    printf("DBG uart_baud=%d\n", MOTOR_UART_BAUDRATE);
    printf("DBG uart_tx_gpio=%d\n", MOTOR_UART_TX_GPIO);
    printf("DBG uart_rx_gpio=%d\n", MOTOR_UART_RX_GPIO);
    printf("DBG gpio tx level=%d\n", gpio_get(MOTOR_UART_TX_GPIO));
    printf("DBG gpio rx level=%d\n", gpio_get(MOTOR_UART_RX_GPIO));
    fflush(stdout);
}

static void log_received_line(const char* line) {
    printf("DBG rx raw=\"");
    for (const unsigned char* p = (const unsigned char*)line; *p != 0; ++p) {
        if (*p == '"' || *p == '\\') {
            printf("\\%c", *p);
        } else if (*p >= 32 && *p <= 126) {
            putchar(*p);
        } else {
            printf("\\x%02X", *p);
        }
    }
    printf("\"\nDBG rx len=%u\n", (unsigned int)strlen(line));
    fflush(stdout);
}

static uint32_t morse_symbol_on_ms(char symbol) {
    return symbol == '-' ? MORSE_DASH_MS : MORSE_DOT_MS;
}

static char* skip_spaces(char* text) {
    while (*text != 0 && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

static void service_led() {
    uint64_t now_us = time_us_64();
    if (led_next_event_us != 0 && now_us < led_next_event_us) {
        return;
    }

    if (!led_active) {
        led_active = true;
        led_symbol_on = true;
        led_pattern_index = 0;
        led_write(true);
        led_next_event_us =
            now_us + morse_symbol_on_ms(current_led_pattern[led_pattern_index]) * 1000;
        return;
    }

    if (led_symbol_on) {
        led_write(false);
        led_symbol_on = false;
        if (current_led_pattern[led_pattern_index + 1] == 0) {
            led_next_event_us = now_us + MORSE_CHAR_GAP_MS * 1000;
        } else {
            led_next_event_us = now_us + MORSE_SYMBOL_GAP_MS * 1000;
        }
        return;
    }

    if (current_led_pattern[led_pattern_index + 1] == 0) {
        led_active = false;
        led_pattern_index = 0;
        led_next_event_us = 0;
        return;
    }

    ++led_pattern_index;
    led_symbol_on = true;
    led_write(true);
    led_next_event_us =
        now_us + morse_symbol_on_ms(current_led_pattern[led_pattern_index]) * 1000;
}

static void wait_ms_with_led(uint32_t ms) {
    absolute_time_t until = make_timeout_time_ms(ms);
    while (!time_reached(until)) {
        service_led();
        sleep_ms(5);
    }
}

static void print_diag_gpio_list(const bool* values) {
    bool any = false;
    for (int gpio = 0; gpio <= DIAG_MAX_GPIO; ++gpio) {
        if (!values[gpio]) {
            continue;
        }
        if (any) {
            printf(",");
        }
        printf("%d", gpio);
        any = true;
    }
    if (!any) {
        printf("none");
    }
}

static void diag_pwm_stop_gpio(int gpio) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, false);
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
    if (diag_gpio_valid(gpio)) {
        diag_pwm[gpio] = false;
        diag_high[gpio] = false;
    }
}

static void diag_gpio_low(int gpio) {
    if (diag_pwm[gpio]) {
        diag_pwm_stop_gpio(gpio);
        return;
    }
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
    diag_high[gpio] = false;
}

static void diag_all_low() {
    for (int gpio = 0; gpio <= DIAG_MAX_GPIO; ++gpio) {
        if (diag_gpio_protected(gpio)) {
            continue;
        }
        if (diag_high[gpio] || diag_pwm[gpio]) {
            diag_gpio_low(gpio);
        }
    }
}

static void print_pin_status() {
    printf("PIN_STATUS PROTECTED=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d "
           "DIAG_HIGH=",
           MOTOR_UART_TX_GPIO, MOTOR_UART_RX_GPIO, MOTOR_SERVO_GPIO,
           MOTOR_PWM0_GPIO, MOTOR_PWM1_GPIO, MOTOR_PWM2_GPIO, MOTOR_PWM3_GPIO,
           MOTOR_QENC0_GPIO, MOTOR_QENC0_GPIO + 1,
           MOTOR_QENC1_GPIO, MOTOR_QENC1_GPIO + 1,
           LED_PIN, 22);
    print_diag_gpio_list(diag_high);
    printf(" DIAG_PWM=");
    print_diag_gpio_list(diag_pwm);
    printf(" MAX_GPIO=%d\n", DIAG_MAX_GPIO);
}

static bool only_spaces(const char* text) {
    while (*text != 0) {
        if (!isspace((unsigned char)*text)) {
            return false;
        }
        ++text;
    }
    return true;
}

static bool parse_one_int(char* text, int* a) {
    int consumed = 0;
    return sscanf(text, "%d %n", a, &consumed) == 1 &&
           only_spaces(text + consumed);
}

static bool parse_two_ints(char* text, int* a, int* b) {
    int consumed = 0;
    return sscanf(text, "%d %d %n", a, b, &consumed) == 2 &&
           only_spaces(text + consumed);
}

static bool parse_three_ints(char* text, int* a, int* b, int* c) {
    int consumed = 0;
    return sscanf(text, "%d %d %d %n", a, b, c, &consumed) == 3 &&
           only_spaces(text + consumed);
}

using firmware::clamp_double;
using firmware::sync_start_boost_target_enabled;

static bool token_equals_ignore_case(const char* a, const char* b) {
    while (*a != 0 && *b != 0) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == 0 && *b == 0;
}

static bool parse_sync_dir(const char* token, int* dir_sign) {
    if (strcmp(token, "+") == 0 || strcmp(token, "1") == 0 ||
        token_equals_ignore_case(token, "PLUS")) {
        *dir_sign = 1;
        return true;
    }
    if (strcmp(token, "-") == 0 || strcmp(token, "-1") == 0 ||
        token_equals_ignore_case(token, "MINUS")) {
        *dir_sign = -1;
        return true;
    }
    return false;
}

static char sync_dir_char(int dir_sign) {
    return dir_sign >= 0 ? '+' : '-';
}

static double encoder_count_to_deg(int count) {
    return (double)count * 360.0 / ENCODER_COUNTS_PER_REV;
}

// Sync API names are physical wheels: left=motor1/enc[1], right=motor0/enc[0].
// Direction +/- is physical wheel rotation; + means CCW viewed from rover side.
static void get_sync_encoder_counts(int* raw_l, int* raw_r, int* dl, int* dr) {
    *raw_l = enc[1].get();
    *raw_r = enc[0].get();
    *dl = *raw_l - sync_state.left_start_count;
    *dr = *raw_r - sync_state.right_start_count;
}

static void sync_cancel(bool stop_motors) {
    sync_state.active = false;
    sync_state.done = false;
    sync_state.ever_started = false;
    sync_state.last_left_speed_cmd_deg_s = 0.0;
    sync_state.last_right_speed_cmd_deg_s = 0.0;
    sync_state.start_boost_enabled = false;
    sync_state.left_start_boost_active = false;
    sync_state.right_start_boost_active = false;
    sync_state.start_boost_start_us = 0;
    sync_state.operation_start_us = 0;
    sync_state.total_timeout_us = 0;
    sync_state.left_last_progress_us = 0;
    sync_state.right_last_progress_us = 0;
    sync_state.error = SYNC_ERROR_NONE;
    sync_state.error_report_pending = false;
    motor_motion_command_active[0] = false;
    motor_motion_command_active[1] = false;
    if (stop_motors && motors_initialized && motor0 != NULL && motor1 != NULL) {
        motor0->resetControlState();
        motor1->resetControlState();
    }
    if (stop_motors) {
        force_motor_outputs_low();
    }
}

static const char* sync_error_string(SyncError error) {
    switch (error) {
        case SYNC_ERROR_NONE:
            return "NONE";
        case SYNC_ERROR_TOTAL_TIMEOUT:
            return "TOTAL_TIMEOUT";
        case SYNC_ERROR_LEFT_STALL:
            return "LEFT_STALL";
        case SYNC_ERROR_RIGHT_STALL:
            return "RIGHT_STALL";
    }
    return "UNKNOWN";
}

static firmware::SyncFaultType sync_fault_type_from_error(SyncError error) {
    switch (error) {
        case SYNC_ERROR_TOTAL_TIMEOUT:
            return firmware::SyncFaultType::kTotalTimeout;
        case SYNC_ERROR_LEFT_STALL:
            return firmware::SyncFaultType::kLeftStall;
        case SYNC_ERROR_RIGHT_STALL:
            return firmware::SyncFaultType::kRightStall;
        case SYNC_ERROR_NONE:
            return firmware::SyncFaultType::kNone;
    }
    return firmware::SyncFaultType::kNone;
}

static int sync_fault_wheel_id(SyncError error) {
    switch (error) {
        case SYNC_ERROR_LEFT_STALL:
            return 1;
        case SYNC_ERROR_RIGHT_STALL:
            return 0;
        default:
            return -1;
    }
}

static void clear_sync_fault_detail() {
    sync_last_fault = {};
}

static void save_sync_fault_detail(SyncError error,
                                   uint64_t now_us,
                                   int left_raw_count,
                                   int right_raw_count,
                                   int left_prev_raw_count,
                                   int right_prev_raw_count,
                                   int left_delta_count,
                                   int right_delta_count,
                                   double left_progress_deg,
                                   double right_progress_deg,
                                   double left_remaining_deg,
                                   double right_remaining_deg,
                                   double left_speed_deg_s,
                                   double right_speed_deg_s,
                                   double correction_deg_s,
                                   double tolerance_deg) {
    firmware::SyncFaultSnapshot snapshot = {};
    snapshot.valid = true;
    snapshot.type = sync_fault_type_from_error(error);
    snapshot.wheel_id = sync_fault_wheel_id(error);
    snapshot.target_deg = sync_state.target_abs_deg;
    snapshot.requested_speed_deg_s = sync_state.base_speed_deg_s;
    snapshot.command_start_us = sync_state.operation_start_us;
    snapshot.fault_us = now_us;
    snapshot.elapsed_ms =
        (now_us - sync_state.operation_start_us) / 1000;
    snapshot.left_raw_count = left_raw_count;
    snapshot.right_raw_count = right_raw_count;
    snapshot.left_prev_raw_count = left_prev_raw_count;
    snapshot.right_prev_raw_count = right_prev_raw_count;
    snapshot.left_delta_count = left_delta_count;
    snapshot.right_delta_count = right_delta_count;
    snapshot.left_progress_deg = left_progress_deg;
    snapshot.right_progress_deg = right_progress_deg;
    snapshot.left_remaining_deg = left_remaining_deg;
    snapshot.right_remaining_deg = right_remaining_deg;
    snapshot.left_speed_deg_s = left_speed_deg_s;
    snapshot.right_speed_deg_s = right_speed_deg_s;
    snapshot.correction_deg_s = correction_deg_s;
    snapshot.left_reached = left_remaining_deg <= tolerance_deg;
    snapshot.right_reached = right_remaining_deg <= tolerance_deg;
    snapshot.left_dir_sign = sync_state.left_dir_sign;
    snapshot.right_dir_sign = sync_state.right_dir_sign;
    snapshot.left_idle_ms =
        (now_us - sync_state.left_last_progress_us) / 1000;
    snapshot.right_idle_ms =
        (now_us - sync_state.right_last_progress_us) / 1000;
    snapshot.left_last_progress_us = sync_state.left_last_progress_us;
    snapshot.right_last_progress_us = sync_state.right_last_progress_us;
    sync_last_fault = snapshot;
}

static void sync_fail(SyncError error) {
    sync_state.active = false;
    sync_state.done = false;
    sync_state.ever_started = true;
    sync_state.error = error;
    sync_state.error_report_pending = true;
    sync_state.last_left_speed_cmd_deg_s = 0.0;
    sync_state.last_right_speed_cmd_deg_s = 0.0;
    sync_state.start_boost_enabled = false;
    sync_state.left_start_boost_active = false;
    sync_state.right_start_boost_active = false;
    motor_motion_command_active[0] = false;
    motor_motion_command_active[1] = false;
    if (motor0 != NULL && motor1 != NULL) {
        motor0->resetControlState();
        motor1->resetControlState();
    }
    force_motor_outputs_low();
}

static void sync_update() {
    if (!sync_state.active || !motors_runtime_enabled || !motors_initialized ||
        motor0 == NULL || motor1 == NULL) {
        return;
    }

    uint64_t now_us = time_us_64();
    int left_raw_count = enc[1].get();
    int right_raw_count = enc[0].get();
    int left_prev_raw_count = sync_state.left_last_raw_count;
    int right_prev_raw_count = sync_state.right_last_raw_count;
    int left_delta_count = left_raw_count - sync_state.left_start_count;
    int right_delta_count = right_raw_count - sync_state.right_start_count;
    int left_directed_progress_count = -sync_state.left_dir_sign * left_delta_count;
    int right_directed_progress_count = -sync_state.right_dir_sign * right_delta_count;
    // Existing position control uses internal motor sign = -physical sign.
    // Apply the same convention so commanded physical motion increases progress.
    double left_progress_deg =
        -sync_state.left_dir_sign * encoder_count_to_deg(left_delta_count);
    double right_progress_deg =
        -sync_state.right_dir_sign * encoder_count_to_deg(right_delta_count);
    double error_deg = left_progress_deg - right_progress_deg;

    sync_state.last_left_progress_deg = left_progress_deg;
    sync_state.last_right_progress_deg = right_progress_deg;
    sync_state.last_error_deg = error_deg;
    if (left_raw_count != sync_state.left_last_raw_count) {
        sync_state.left_last_raw_count = left_raw_count;
        sync_state.left_last_progress_us = now_us;
    }
    if (right_raw_count != sync_state.right_last_raw_count) {
        sync_state.right_last_raw_count = right_raw_count;
        sync_state.right_last_progress_us = now_us;
    }

    uint64_t elapsed_boost_us =
        now_us - sync_state.start_boost_start_us;
    firmware::SyncSpeedResult speeds =
        firmware::calculate_sync_speeds(
            sync_state.target_abs_deg,
            left_progress_deg,
            right_progress_deg,
            sync_state.base_speed_deg_s,
            sync_state.left_start_boost_active,
            sync_state.right_start_boost_active,
            left_directed_progress_count,
            right_directed_progress_count,
            elapsed_boost_us);

    if (speeds.done) {
        motor0->setVel(0);
        motor1->setVel(0);
        sync_state.last_left_speed_cmd_deg_s = 0.0;
        sync_state.last_right_speed_cmd_deg_s = 0.0;
        sync_state.final_left_progress_deg = left_progress_deg;
        sync_state.final_right_progress_deg = right_progress_deg;
        sync_state.final_error_deg = error_deg;
        sync_state.active = false;
        sync_state.done = true;
        sync_state.start_boost_enabled = false;
        sync_state.left_start_boost_active = false;
        sync_state.right_start_boost_active = false;
        sync_state.start_boost_start_us = 0;
        sync_state.error = SYNC_ERROR_NONE;
        motor_motion_command_active[0] = false;
        motor_motion_command_active[1] = false;
        return;
    }

    double left_remaining = sync_state.target_abs_deg - left_progress_deg;
    double right_remaining = sync_state.target_abs_deg - right_progress_deg;
    double correction_deg_s =
        firmware::kSyncKp * (left_progress_deg - right_progress_deg);
    if (firmware::deadline_expired(
            now_us,
            sync_state.operation_start_us,
            sync_state.total_timeout_us)) {
        save_sync_fault_detail(SYNC_ERROR_TOTAL_TIMEOUT,
                               now_us,
                               left_raw_count,
                               right_raw_count,
                               left_prev_raw_count,
                               right_prev_raw_count,
                               left_delta_count,
                               right_delta_count,
                               left_progress_deg,
                               right_progress_deg,
                               left_remaining,
                               right_remaining,
                               sync_state.left_dir_sign * speeds.left_abs,
                               sync_state.right_dir_sign * speeds.right_abs,
                               correction_deg_s,
                               speeds.tolerance);
        sync_fail(SYNC_ERROR_TOTAL_TIMEOUT);
        return;
    }
    if (left_remaining > speeds.tolerance &&
        firmware::deadline_expired(
            now_us,
            sync_state.left_last_progress_us,
            firmware::kSyncStallTimeoutUs)) {
        save_sync_fault_detail(SYNC_ERROR_LEFT_STALL,
                               now_us,
                               left_raw_count,
                               right_raw_count,
                               left_prev_raw_count,
                               right_prev_raw_count,
                               left_delta_count,
                               right_delta_count,
                               left_progress_deg,
                               right_progress_deg,
                               left_remaining,
                               right_remaining,
                               sync_state.left_dir_sign * speeds.left_abs,
                               sync_state.right_dir_sign * speeds.right_abs,
                               correction_deg_s,
                               speeds.tolerance);
        sync_fail(SYNC_ERROR_LEFT_STALL);
        return;
    }
    if (right_remaining > speeds.tolerance &&
        firmware::deadline_expired(
            now_us,
            sync_state.right_last_progress_us,
            firmware::kSyncStallTimeoutUs)) {
        save_sync_fault_detail(SYNC_ERROR_RIGHT_STALL,
                               now_us,
                               left_raw_count,
                               right_raw_count,
                               left_prev_raw_count,
                               right_prev_raw_count,
                               left_delta_count,
                               right_delta_count,
                               left_progress_deg,
                               right_progress_deg,
                               left_remaining,
                               right_remaining,
                               sync_state.left_dir_sign * speeds.left_abs,
                               sync_state.right_dir_sign * speeds.right_abs,
                               correction_deg_s,
                               speeds.tolerance);
        sync_fail(SYNC_ERROR_RIGHT_STALL);
        return;
    }
    sync_state.left_start_boost_active = speeds.left_boost_active;
    sync_state.right_start_boost_active = speeds.right_boost_active;
    double left_speed_abs = speeds.left_abs;
    double right_speed_abs = speeds.right_abs;
    double left_physical_speed = sync_state.left_dir_sign * left_speed_abs;
    double right_physical_speed = sync_state.right_dir_sign * right_speed_abs;
    double left_internal_cmd = -left_physical_speed;
    double right_internal_cmd = -right_physical_speed;

    sync_state.last_left_speed_cmd_deg_s = left_physical_speed;
    sync_state.last_right_speed_cmd_deg_s = right_physical_speed;
    motor1->setVel((float)left_internal_cmd);
    motor0->setVel((float)right_internal_cmd);
}

static void print_sync_status() {
    if (sync_state.active) {
        sync_update();
    }

    int raw_l = 0;
    int raw_r = 0;
    int dl = 0;
    int dr = 0;
    bool have_encoder_counts = motors_initialized;
    if (have_encoder_counts) {
        get_sync_encoder_counts(&raw_l, &raw_r, &dl, &dr);
    }

    if (sync_state.active) {
        printf("SYNC_STATUS state=BUSY target=%.3f left=%.3f right=%.3f "
               "error=%.3f speed_l=%.3f speed_r=%.3f raw_l=%d raw_r=%d dl=%d dr=%d\n",
               sync_state.target_abs_deg,
               sync_state.last_left_progress_deg,
               sync_state.last_right_progress_deg,
               sync_state.last_error_deg,
               sync_state.last_left_speed_cmd_deg_s,
               sync_state.last_right_speed_cmd_deg_s,
               raw_l,
               raw_r,
               dl,
               dr);
        return;
    }
    if (sync_state.done && sync_state.ever_started) {
        printf("SYNC_STATUS state=DONE target=%.3f left=%.3f right=%.3f "
               "error=%.3f raw_l=%d raw_r=%d dl=%d dr=%d\n",
               sync_state.target_abs_deg,
               sync_state.final_left_progress_deg,
               sync_state.final_right_progress_deg,
               sync_state.final_error_deg,
               raw_l,
               raw_r,
               dl,
               dr);
        return;
    }
    if (sync_state.error != SYNC_ERROR_NONE && sync_state.ever_started) {
        printf("SYNC_STATUS state=ERROR error=%s target=%.3f "
               "left=%.3f right=%.3f raw_l=%d raw_r=%d dl=%d dr=%d\n",
               sync_error_string(sync_state.error),
               sync_state.target_abs_deg,
               sync_state.last_left_progress_deg,
               sync_state.last_right_progress_deg,
               raw_l,
               raw_r,
               dl,
               dr);
        return;
    }
    if (have_encoder_counts) {
        printf("SYNC_STATUS state=IDLE raw_l=%d raw_r=%d\n", raw_l, raw_r);
        return;
    }
    printf("SYNC_STATUS state=IDLE\n");
}

static void print_sync_fault_status() {
    char line[1200];
    if (!firmware::format_sync_fault_status(
            line, sizeof(line), sync_last_fault)) {
        printf("SYNC_FAULT state=FORMAT_ERROR\n");
        return;
    }
    printf("%s\n", line);
}

static void handle_motors_sync_rot(char* rest) {
    double abs_deg = 0.0;
    double speed_deg_s = 0.0;
    char left_dir_token[16];
    char right_dir_token[16];
    int left_dir_sign = 0;
    int right_dir_sign = 0;
    int consumed = 0;
    if (sscanf(rest, "%lf %15s %15s %lf %n",
               &abs_deg, left_dir_token, right_dir_token,
               &speed_deg_s, &consumed) != 4 ||
        !only_spaces(rest + consumed) ||
        !std::isfinite(abs_deg) || !std::isfinite(speed_deg_s)) {
        safe_all();
        printf("ERR INVALID_SYNC_TARGET\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!motors_runtime_enabled) {
        force_motor_outputs_low();
        printf("ERR MOTORS_DISABLED\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (abs_deg <= 0.0) {
        force_motor_outputs_low();
        printf("ERR INVALID_SYNC_TARGET\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (speed_deg_s <= 0.0) {
        force_motor_outputs_low();
        printf("ERR INVALID_SYNC_SPEED\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!parse_sync_dir(left_dir_token, &left_dir_sign) ||
        !parse_sync_dir(right_dir_token, &right_dir_sign)) {
        force_motor_outputs_low();
        printf("ERR INVALID_SYNC_DIR\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!motors_initialized || motor0 == NULL || motor1 == NULL) {
        force_motor_outputs_low();
        printf("ERR MOTORS_DISABLED\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (sync_state.active) {
        printf("ERR SYNC_BUSY\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }

    sync_start_initializing = true;
    sync_state.active = false;

    motor0->resetControlState();
    motor1->resetControlState();
    force_motor_outputs_low();

    sync_state.done = false;
    sync_state.ever_started = true;
    sync_state.left_start_count = enc[1].get();
    sync_state.right_start_count = enc[0].get();
    sync_state.target_abs_deg = abs_deg;
    sync_state.left_dir_sign = left_dir_sign;
    sync_state.right_dir_sign = right_dir_sign;
    sync_state.base_speed_deg_s =
        clamp_double(speed_deg_s, 0.0, firmware::kSyncMaxSpeedDegS);
    sync_state.last_left_progress_deg = 0.0;
    sync_state.last_right_progress_deg = 0.0;
    sync_state.last_error_deg = 0.0;
    sync_state.last_left_speed_cmd_deg_s = 0.0;
    sync_state.last_right_speed_cmd_deg_s = 0.0;
    sync_state.final_left_progress_deg = 0.0;
    sync_state.final_right_progress_deg = 0.0;
    sync_state.final_error_deg = 0.0;
    sync_state.start_boost_enabled = sync_start_boost_target_enabled(abs_deg);
    sync_state.left_start_boost_active = sync_state.start_boost_enabled;
    sync_state.right_start_boost_active = sync_state.start_boost_enabled;
    sync_state.start_boost_start_us = 0;
    sync_state.error = SYNC_ERROR_NONE;
    sync_state.error_report_pending = false;

    if (!restore_motor_pwm_outputs()) {
        sync_start_initializing = false;
        safe_all();
        printf("ERR MOTOR_PWM_RESTORE_FAILED\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    motor0->resetControlState();
    motor1->resetControlState();
    uint64_t start_us = time_us_64();
    sync_state.start_boost_start_us = start_us;
    sync_state.operation_start_us = start_us;
    sync_state.total_timeout_us =
        firmware::calculate_sync_total_timeout_us(abs_deg, speed_deg_s);
    sync_state.left_last_progress_us = start_us;
    sync_state.right_last_progress_us = start_us;
    sync_state.left_last_raw_count = sync_state.left_start_count;
    sync_state.right_last_raw_count = sync_state.right_start_count;
    sync_state.active = true;
    motor_motion_command_active[0] = true;
    motor_motion_command_active[1] = true;
    sync_start_initializing = false;
    sync_update();

    printf("OK MOTORS_SYNC_ROT target=%.3f left_dir=%c right_dir=%c speed=%.3f\n",
           sync_state.target_abs_deg,
           sync_dir_char(sync_state.left_dir_sign),
           sync_dir_char(sync_state.right_dir_sign),
           sync_state.base_speed_deg_s);
    fflush(stdout);
    set_led_char('M');
}

static void handle_gpio_read(char* rest) {
    int gpio = -1;
    if (!parse_one_int(rest, &gpio)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }

    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    diag_high[gpio] = false;
    diag_pwm[gpio] = false;
    printf("GPIO %d LEVEL=%d\n", gpio, gpio_get(gpio) ? 1 : 0);
    fflush(stdout);
    set_led_char('D');
}

static void handle_gpio_high(char* rest) {
    int gpio = -1;
    if (!parse_one_int(rest, &gpio)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }

    if (diag_pwm[gpio]) {
        diag_pwm_stop_gpio(gpio);
    }
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    diag_high[gpio] = true;
    printf("OK GPIO_HIGH %d\n", gpio);
    fflush(stdout);
    set_led_char('D');
}

static void handle_gpio_low(char* rest) {
    int gpio = -1;
    if (!parse_one_int(rest, &gpio)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }

    diag_gpio_low(gpio);
    printf("OK GPIO_LOW %d\n", gpio);
    fflush(stdout);
    set_led_char('D');
}

static void handle_gpio_pulse(char* rest) {
    int gpio = -1;
    int ms = 0;
    if (!parse_two_ints(rest, &gpio, &ms)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }
    if (ms <= 0 || ms > 1000) {
        printf("ERR INVALID_MS\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }

    if (diag_pwm[gpio]) {
        diag_pwm_stop_gpio(gpio);
    }
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    diag_high[gpio] = true;
    wait_ms_with_led((uint32_t)ms);
    gpio_put(gpio, 0);
    diag_high[gpio] = false;
    printf("OK GPIO_PULSE %d %d\n", gpio, ms);
    fflush(stdout);
    set_led_char('D');
}

static void handle_pwm_test(char* rest) {
    int gpio = -1;
    int duty_percent = 0;
    int ms = 0;
    if (!parse_three_ints(rest, &gpio, &duty_percent, &ms)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }
    if (duty_percent < 0 || duty_percent > 50) {
        printf("ERR INVALID_DUTY\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (ms <= 0 || ms > 1000) {
        printf("ERR INVALID_MS\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }

    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    firmware::PwmTiming timing =
        firmware::calculate_pwm_timing(
            clock_get_hz(clk_sys), DIAG_PWM_FREQ);
    if (!timing.valid) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
        gpio_put(gpio, 0);
        printf("ERR PWM_CONFIG\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    pwm_set_clkdiv_int_frac(
        slice, timing.divider_integer, timing.divider_fraction_16);
    pwm_set_wrap(slice, timing.wrap);
    pwm_set_chan_level(
        slice, channel,
        static_cast<uint16_t>(
            static_cast<uint32_t>(timing.wrap) * duty_percent / 100));
    pwm_set_enabled(slice, true);
    diag_pwm[gpio] = true;
    diag_high[gpio] = false;
    wait_ms_with_led((uint32_t)ms);
    diag_pwm_stop_gpio(gpio);
    printf("OK PWM_TEST %d duty=%d ms=%d\n", gpio, duty_percent, ms);
    fflush(stdout);
    set_led_char('D');
}

static void handle_pwm_off(char* rest) {
    int gpio = -1;
    if (!parse_one_int(rest, &gpio)) {
        safe_all();
        printf("ERR INVALID_GPIO\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }
    if (!diag_runtime_available() || !diag_validate_gpio(gpio)) {
        return;
    }

    diag_pwm_stop_gpio(gpio);
    printf("OK PWM_OFF %d\n", gpio);
    fflush(stdout);
    set_led_char('D');
}

static bool timer_cb(repeating_timer_t* rt) {
    if (sync_start_initializing || !motors_runtime_enabled || !motors_initialized ||
        motor0 == NULL || motor1 == NULL) {
        return timer_started;
    }
    sync_update();
    motor0->timer_cb();
    motor1->timer_cb();
    return timer_started;
}

static bool timer_cb_pos(repeating_timer_t* rt) {
    if (sync_start_initializing || !motors_runtime_enabled || !motors_initialized ||
        motor0 == NULL || motor1 == NULL) {
        return timer_started;
    }
    motor0->timer_cb_pos();
    motor1->timer_cb_pos();
    return timer_started;
}

static bool start_motor_timers() {
    if (timer_started) {
        return true;
    }
    bool velocity_ok =
        add_repeating_timer_ms(-10, timer_cb, NULL, &velocity_timer);
    if (!velocity_ok) {
        return false;
    }
    bool position_ok =
        add_repeating_timer_ms(-100, timer_cb_pos, NULL, &position_timer);
    if (!firmware::timer_registration_succeeded(
            velocity_ok, position_ok)) {
        cancel_repeating_timer(&velocity_timer);
        return false;
    }
    timer_started = true;
    return true;
}

static void stop_motor_timers() {
    if (!timer_started) {
        return;
    }
    cancel_repeating_timer(&velocity_timer);
    cancel_repeating_timer(&position_timer);
    timer_started = false;
}

static void stop_motors_if_enabled() {
    if (!motors_initialized || motor0 == NULL || motor1 == NULL) {
        force_motor_outputs_low();
        return;
    }
    sync_cancel(false);
    motor0->resetControlState();
    motor1->resetControlState();
    motor_motion_command_active[0] = false;
    motor_motion_command_active[1] = false;
    force_motor_outputs_low();
}

static void configure_motor_gains() {
    if (motor0 == NULL || motor1 == NULL) {
        return;
    }
    motor0->setVelGain(1, 0.0, 0.09);
    motor0->setPosGain(2.5, 0.0, 0.09);
    motor1->setVelGain(1, 0.0, 0.09);
    motor1->setPosGain(2.5, 0.0, 0.09);
}

static bool enable_motors() {
    force_motor_outputs_low();
    motor_enable_error = "NONE";
    if (motors_pin_conflict()) {
        motor_enable_error = "PIN_CONFLICT";
        force_motor_outputs_low();
        return false;
    }
    if (!create_motor_objects_if_needed()) {
        motor_enable_error = "ALLOCATION_FAILED";
        force_motor_outputs_low();
        return false;
    }
    force_motor_outputs_low();

    if (!motors_initialized) {
        gpio_set_dir(MOTOR_QENC0_GPIO, GPIO_IN);
        gpio_set_dir(MOTOR_QENC0_GPIO + 1, GPIO_IN);
        gpio_set_dir(MOTOR_QENC1_GPIO, GPIO_IN);
        gpio_set_dir(MOTOR_QENC1_GPIO + 1, GPIO_IN);
        motor0->init();
        motor1->init();
        if (!pwm0->isInitialized() || !pwm1->isInitialized() ||
            !pwm2->isInitialized() || !pwm3->isInitialized()) {
            motor_enable_error = "PWM_CONFIG_FAILED";
            force_motor_outputs_low();
            return false;
        }
        configure_motor_gains();
        motors_initialized = true;
    } else {
        configure_motor_gains();
    }

    motors_runtime_enabled = true;
    stop_motors_if_enabled();
    motor0->resetControlState();
    motor1->resetControlState();
    if (!start_motor_timers()) {
        motor_enable_error = "TIMER_START_FAILED";
        motors_runtime_enabled = false;
        stop_motor_timers();
        motor0->resetControlState();
        motor1->resetControlState();
        force_motor_outputs_low();
        return false;
    }
    force_motor_outputs_low();
    return true;
}

static void disable_motors() {
    // Prevent both callbacks from re-entering motor control while control state
    // and pin muxes are being returned to the physical LOW safe state.
    motors_runtime_enabled = false;
    stop_motor_timers();
    sync_cancel(true);
    stop_motors_if_enabled();
    motor_motion_command_active[0] = false;
    motor_motion_command_active[1] = false;
    force_motor_outputs_low();
}

static bool enable_servo() {
    if (servo_pin_conflict()) {
        return false;
    }
    if (servo_runtime_enabled) {
        return true;
    }
    if (!servo_output.enable()) {
        servo_runtime_enabled = false;
        return false;
    }
    servo_runtime_enabled = true;
    return true;
}

static void disable_servo() {
    servo_output.disable();
    servo_runtime_enabled = false;
}

static void safe_stop_motors_callback() {
    disable_motors();
}

static void safe_stop_servo_callback() {
    disable_servo();
}

static void safe_stop_diagnostics_callback() {
    diag_all_low();
}

static void safe_all() {
    firmware::SafeStopCallbacks callbacks = {
        safe_stop_motors_callback,
        safe_stop_servo_callback,
        safe_stop_diagnostics_callback};
    firmware::invoke_safe_stop(callbacks);
    motor_motion_command_active[0] = false;
    motor_motion_command_active[1] = false;
    force_motor_outputs_low();
}

static void print_status() {
    printf("FW=NORMAL UART_ID=%d TX=%d RX=%d BAUD=%d "
           "MOTORS_ENABLED=%d SERVO_ENABLED=%d SERVO_PULSING=%d "
           "SERVO_PULSE_US=%u SERVO_PIO_SM=%d TIMER_STARTED=%d "
           "PWM=%d,%d,%d,%d SERVO_GPIO=%d QENC=%d,%d LED=%c PIN_CONFLICT=",
           MOTOR_UART_ID,
           MOTOR_UART_TX_GPIO,
           MOTOR_UART_RX_GPIO,
           MOTOR_UART_BAUDRATE,
           motors_runtime_enabled ? 1 : 0,
           servo_runtime_enabled ? 1 : 0,
           servo_output.is_pulsing() ? 1 : 0,
           servo_output.applied_pulse_us(),
           servo_output.state_machine(),
           timer_started ? 1 : 0,
           MOTOR_PWM0_GPIO,
           MOTOR_PWM1_GPIO,
           MOTOR_PWM2_GPIO,
           MOTOR_PWM3_GPIO,
           MOTOR_SERVO_GPIO,
           MOTOR_QENC0_GPIO,
           MOTOR_QENC1_GPIO,
           current_led_char);
    print_pin_conflict_value();
    printf(" DIAG_HIGH=");
    print_diag_gpio_list(diag_high);
    printf(" DIAG_PWM=");
    print_diag_gpio_list(diag_pwm);
    printf("\n");
}

static void print_encoder_status() {
    if (!motors_runtime_enabled) {
        printf("ERR MOTORS_DISABLED\n");
        fflush(stdout);
        return;
    }

    // Keep this aligned with the physical sync API mapping below.
    printf("ENCODER left=%d right=%d\n", enc[1].get(), enc[0].get());
    fflush(stdout);
}

static void print_help() {
    printf("COMMANDS: PING, STATUS, STOP, SAFE, MOTOR_ENABLE, MOTOR_DISABLE, "
           "SERVO_ENABLE, SERVO_DISABLE, LED <B|U|O|A|N|M|S|L|R|P|E|I|T|X>, "
           "MOTORS_SYNC_ROT <abs_deg> <left_dir> <right_dir> <speed_deg_s>, "
           "MOTORS_SYNC_STATUS, MOTORS_SYNC_FAULT_STATUS, "
           "MOTORS_SYNC_FAULT_CLEAR, MOTORS_SYNC_CANCEL, ENCODER, "
           "PIN_STATUS, GPIO_READ <gpio>, GPIO_HIGH <gpio>, GPIO_LOW <gpio>, "
           "GPIO_PULSE <gpio> <ms>, PWM_TEST <gpio> <duty> <ms>, PWM_OFF <gpio>, "
           "DIAG_ALL_LOW, motor commands\n");
    printf("MOTOR: <id> <mode> <val>\n");
    printf("SERVO: 2 0 <pulse_command_deg 0..180>; "
           "90 is provisional neutral, not physical angle\n");
    printf("ENCODER: print encoder counts as ENCODER left=<count0> right=<count1>\n");
    printf("MOTORS_SYNC_ROT: left/right are physical wheels; "
           "left=motor1/enc1 right=motor0/enc0; + is CCW from rover side\n");
    printf("MOTORS_SYNC_FAULT_STATUS: print last sync fault detail; "
           "MOTORS_SYNC_FAULT_CLEAR clears it\n");
}

static bool reject_unexpected_args(const char* command, char* rest) {
    if (only_spaces(rest)) {
        return false;
    }
    safe_all();
    set_led_char('E');
    printf("ERR %s_ARGS\n", command);
    fflush(stdout);
    return true;
}

static bool handle_text_command(char* line) {
    char command[firmware::kTextCommandTokenBufferSize];
    firmware::TextCommandToken token =
        firmware::read_text_command_token(line, command, sizeof(command));
    char* rest = token.rest;
    if (command[0] == 0) {
        return true;
    }
    if (!firmware::is_text_command_token(command)) {
        return false;
    }

    if (strcmp(command, "LED") == 0) {
        rest = skip_spaces(rest);
        if (*rest == 0) {
            safe_all();
            set_led_char('I');
            printf("ERR LED missing\n");
            fflush(stdout);
            return true;
        }

        char led_char = (char)toupper((unsigned char)*rest);
        char* led_tail = rest + 1;
        if (!only_spaces(led_tail)) {
            safe_all();
            set_led_char('E');
            printf("ERR LED_ARGS\n");
            fflush(stdout);
            return true;
        }
        if (!set_led_char(led_char)) {
            safe_all();
            set_led_char('I');
            printf("ERR LED unsupported char=%c\n", led_char);
            fflush(stdout);
            return true;
        }

        printf("OK LED %c\n", current_led_char);
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "PING") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        set_led_char('P');
        printf("PONG\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "STATUS") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        print_status();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "HELP") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        print_help();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "PIN_STATUS") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        print_pin_status();
        fflush(stdout);
        set_led_char('D');
        return true;
    }

    if (strcmp(command, "ENCODER") == 0) {
        if (*skip_spaces(rest) != 0) {
            safe_all();
            printf("ERR ENCODER_ARGS\n");
            fflush(stdout);
            set_led_char('E');
            return true;
        }
        print_encoder_status();
        return true;
    }

    if (strcmp(command, "GPIO_READ") == 0) {
        handle_gpio_read(rest);
        return true;
    }

    if (strcmp(command, "GPIO_HIGH") == 0) {
        handle_gpio_high(rest);
        return true;
    }

    if (strcmp(command, "GPIO_LOW") == 0) {
        handle_gpio_low(rest);
        return true;
    }

    if (strcmp(command, "GPIO_PULSE") == 0) {
        handle_gpio_pulse(rest);
        return true;
    }

    if (strcmp(command, "PWM_TEST") == 0) {
        handle_pwm_test(rest);
        return true;
    }

    if (strcmp(command, "PWM_OFF") == 0) {
        handle_pwm_off(rest);
        return true;
    }

    if (strcmp(command, "DIAG_ALL_LOW") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        if (!diag_runtime_available()) {
            return true;
        }
        diag_all_low();
        printf("OK DIAG_ALL_LOW\n");
        fflush(stdout);
        set_led_char('D');
        return true;
    }

    if (strcmp(command, "MOTORS_SYNC_ROT") == 0) {
        handle_motors_sync_rot(rest);
        return true;
    }

    if (strcmp(command, "MOTORS_SYNC_STATUS") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        print_sync_status();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTORS_SYNC_FAULT_STATUS") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        print_sync_fault_status();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTORS_SYNC_FAULT_CLEAR") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        clear_sync_fault_detail();
        printf("%s\n", firmware::kMotorsSyncFaultClearAck);
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTORS_SYNC_CANCEL") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        sync_cancel(true);
        set_led_char('S');
        printf("OK MOTORS_SYNC_CANCEL\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "STOP") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        stop_motors_if_enabled();
        set_led_char('S');
        printf("OK STOP\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SAFE") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        safe_all();
        set_led_char('S');
        printf("OK SAFE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTOR_ENABLE") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        if (!enable_motors()) {
            set_led_char('E');
            printf("ERR MOTOR_ENABLE %s\n", motor_enable_error);
            fflush(stdout);
            return true;
        }
        set_led_char('M');
        printf("OK MOTOR_ENABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTOR_DISABLE") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        disable_motors();
        set_led_char('S');
        printf("OK MOTOR_DISABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SERVO_ENABLE") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        if (!enable_servo()) {
            set_led_char('E');
            printf("ERR SERVO_ENABLE %s\n",
                   servo_pin_conflict()
                       ? "PIN_CONFLICT"
                       : servo_output.last_error());
            fflush(stdout);
            return true;
        }
        set_led_char('A');
        printf("OK SERVO_ENABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SERVO_DISABLE") == 0) {
        if (reject_unexpected_args(command, rest)) {
            return true;
        }
        disable_servo();
        set_led_char('S');
        printf("OK SERVO_DISABLE\n");
        fflush(stdout);
        return true;
    }

    return false;
}

static void service_runtime_safety() {
    if (sync_state.error_report_pending) {
        SyncError error = sync_state.error;
        sync_state.error_report_pending = false;
        set_led_char('E');
        printf("ERR SYNC_%s\n", sync_error_string(error));
        fflush(stdout);
    }

    if (servo_output.has_feed_fault()) {
        safe_all();
        set_led_char('E');
        printf("ERR SERVO_PIO_FEED_FAILED\n");
        fflush(stdout);
        return;
    }

    bool motion_active =
        motor_motion_command_active[0] ||
        motor_motion_command_active[1] ||
        sync_state.active ||
        servo_output.is_pulsing();
    uint64_t now_us = time_us_64();
    if (motion_active &&
        firmware::deadline_expired(
            now_us,
            last_complete_command_us,
            firmware::kCommandWatchdogTimeoutUs)) {
        safe_all();
        watchdog_report_pending = true;
    }
    if (watchdog_report_pending) {
        watchdog_report_pending = false;
        set_led_char('E');
        printf("ERR COMMAND_WATCHDOG_TIMEOUT\n");
        fflush(stdout);
    }
}

enum ReadLineResult {
    READ_LINE_OK = 0,
    READ_LINE_TOO_LONG,
    READ_LINE_TIMEOUT,
};

static ReadLineResult readline(char* line, int line_size) {
    int i = 0;
    uint64_t partial_line_start_us = 0;
    while (true) {
        int c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
        service_led();
        service_runtime_safety();
        if (c_raw == PICO_ERROR_TIMEOUT) {
            if (partial_line_start_us != 0 &&
                firmware::deadline_expired(
                    time_us_64(),
                    partial_line_start_us,
                    firmware::kUartPartialLineTimeoutUs)) {
                line[i] = 0;
                return READ_LINE_TIMEOUT;
            }
            continue;
        }

        char c = (char)c_raw;
        if (c == '\n') {
            break;
        }
        if (c == '\r') {
            continue;
        }
        if (partial_line_start_us == 0) {
            partial_line_start_us = time_us_64();
        }
        if (i >= line_size - 1) {
            line[line_size - 1] = 0;
            while (c != '\n') {
                c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
                service_led();
                service_runtime_safety();
                if (c_raw == PICO_ERROR_TIMEOUT) {
                    if (firmware::deadline_expired(
                            time_us_64(),
                            partial_line_start_us,
                            firmware::kUartPartialLineTimeoutUs)) {
                        return READ_LINE_TIMEOUT;
                    }
                    continue;
                }
                c = (char)c_raw;
            }
            return READ_LINE_TOO_LONG;
        }
        line[i++] = c;
    }
    line[i] = 0;
    return READ_LINE_OK;
}

int main() {
    force_motor_outputs_low();
    servo_output.disable();
    init_led();
    init_uart_stdio();
    set_led_char('L');
    printf("FW NORMAL SAFE BOOT\n");
    log_uart_config();
    last_complete_command_us = time_us_64();

    while (true) {
        service_led();
        ReadLineResult read_result = readline(buf, sizeof(buf));
        if (read_result != READ_LINE_OK) {
            safe_all();
            set_led_char('E');
            printf("ERR %s\n",
                   read_result == READ_LINE_TOO_LONG
                       ? "LINE_TOO_LONG"
                       : "PARTIAL_LINE_TIMEOUT");
            fflush(stdout);
            continue;
        }

        last_complete_command_us = time_us_64();
        set_led_char('R');
        log_received_line(buf);
        if (handle_text_command(buf)) {
            continue;
        }

        firmware::NumericCommand command = {};
        firmware::NumericParseError parse_error =
            firmware::NumericParseError::kNone;
        if (!firmware::parse_numeric_command(
                buf, &command, &parse_error)) {
            safe_all();
            set_led_char('E');
            printf("ERR PARSE_%s raw=\"%s\"\n",
                   firmware::numeric_parse_error_string(parse_error),
                   buf);
            fflush(stdout);
            continue;
        }

        int id = command.id;
        int mode = command.mode;
        double val = command.value;
        printf("DBG parse n=3 id=%d mode=%d val=%.3f\n", id, mode, val);
        printf("DBG dispatch id=%d mode=%d val=%.3f\n", id, mode, val);
        fflush(stdout);

        if (id < 0 || id > 2) {
            safe_all();
            set_led_char('I');
            printf("ERR id out_of_range id=%d\n", id);
            fflush(stdout);
            continue;
        }
        if ((id == 0 || id == 1) &&
            (!motors_runtime_enabled || !motors_initialized ||
             motor0 == NULL || motor1 == NULL)) {
            force_motor_outputs_low();
            set_led_char('E');
            printf("ERR MOTORS_DISABLED\n");
            fflush(stdout);
            continue;
        }
        if ((id == 0 || id == 1) && sync_state.active) {
            force_motor_outputs_low();
            set_led_char('E');
            printf("ERR SYNC_BUSY\n");
            fflush(stdout);
            continue;
        }
        if (id == 2 && !servo_runtime_enabled) {
            set_led_char('E');
            printf("ERR SERVO_DISABLED\n");
            fflush(stdout);
            continue;
        }
        if ((id == 0 || id == 1) && mode != 0 && mode != 1) {
            safe_all();
            set_led_char('I');
            printf("ERR mode invalid id=%d mode=%d\n", id, mode);
            fflush(stdout);
            continue;
        }
        if (id == 2) {
            firmware::ServoCommandError servo_error =
                firmware::validate_servo_command(mode, val);
            if (servo_error != firmware::ServoCommandError::kNone) {
                set_led_char('E');
                printf("ERR SERVO_%s\n",
                       firmware::servo_command_error_string(servo_error));
                fflush(stdout);
                continue;
            }
        }

        switch (id) {
            case 0:
                sync_cancel(false);
                if (!restore_motor_pwm_outputs()) {
                    safe_all();
                    set_led_char('E');
                    printf("ERR MOTOR_PWM_RESTORE_FAILED\n");
                    fflush(stdout);
                    continue;
                }
                if (!mode) {
                    motor0->disablePosPid();
                    motor0->setVel(val);
                    printf("OK motor id=0 mode=vel target=%.3f\n", val);
                } else {
                    motor0->resetPos();
                    motor0->setPos(val);
                    printf("OK motor id=0 mode=pos target=%.3f\n", val);
                }
                motor_motion_command_active[0] =
                    mode != 0 || val != 0.0;
                break;
            case 1:
                sync_cancel(false);
                if (!restore_motor_pwm_outputs()) {
                    safe_all();
                    set_led_char('E');
                    printf("ERR MOTOR_PWM_RESTORE_FAILED\n");
                    fflush(stdout);
                    continue;
                }
                if (!mode) {
                    motor1->disablePosPid();
                    motor1->setVel(val);
                    printf("OK motor id=1 mode=vel target=%.3f\n", val);
                } else {
                    motor1->resetPos();
                    motor1->setPos(val);
                    printf("OK motor id=1 mode=pos target=%.3f\n", val);
                }
                motor_motion_command_active[1] =
                    mode != 0 || val != 0.0;
                break;
            case 2: {
                uint32_t pulse_us =
                    firmware::servo_command_to_pulse_us(val);
                if (!servo_output.write_pulse_us(pulse_us)) {
                    const char* error = servo_output.last_error();
                    safe_all();
                    set_led_char('E');
                    printf("ERR SERVO_OUTPUT %s\n", error);
                    fflush(stdout);
                    continue;
                }
                char acknowledgement[96];
                if (!firmware::format_servo_ack(
                        acknowledgement,
                        sizeof(acknowledgement),
                        mode,
                        val,
                        servo_output.applied_pulse_us())) {
                    safe_all();
                    set_led_char('E');
                    printf("ERR SERVO_ACK_FORMAT\n");
                    fflush(stdout);
                    continue;
                }
                printf("%s\n", acknowledgement);
                break;
            }
        }
        set_led_char('T');
        fflush(stdout);
    }
}
