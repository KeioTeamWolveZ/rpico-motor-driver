#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "../lib/rpico-encoder-plus/qenc.h"
#include "../lib/rpico-motor/motor.h"
#include "../lib/rpico-pwm/pwm.h"
#include "../lib/rpico-servo/servo.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"

#ifndef MOTOR_UART_ID
#define MOTOR_UART_ID 0
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
#define MOTOR_SERVO_GPIO 18
#endif

#ifndef MOTOR_PWM0_GPIO
#define MOTOR_PWM0_GPIO 6
#endif

#ifndef MOTOR_PWM1_GPIO
#define MOTOR_PWM1_GPIO 7
#endif

#ifndef MOTOR_PWM2_GPIO
#define MOTOR_PWM2_GPIO 2
#endif

#ifndef MOTOR_PWM3_GPIO
#define MOTOR_PWM3_GPIO 3
#endif

#ifndef MOTOR_QENC0_GPIO
#define MOTOR_QENC0_GPIO 26
#endif

#ifndef MOTOR_QENC1_GPIO
#define MOTOR_QENC1_GPIO 28
#endif

#if MOTOR_UART_ID == 0
#define MOTOR_UART_INSTANCE uart0
#elif MOTOR_UART_ID == 1
#define MOTOR_UART_INSTANCE uart1
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

Pwm pwm[4] = {
    Pwm(MOTOR_PWM0_GPIO, 50000),
    Pwm(MOTOR_PWM1_GPIO, 50000),
    Pwm(MOTOR_PWM2_GPIO, 50000),
    Pwm(MOTOR_PWM3_GPIO, 50000),
};
Servo servo(MOTOR_SERVO_GPIO);
Qenc enc[2] = {Qenc(MOTOR_QENC0_GPIO), Qenc(MOTOR_QENC1_GPIO)};
Motor motor[2] = {Motor(pwm[3], pwm[0], enc[0]), Motor(pwm[1], pwm[2], enc[1])};

static char buf[255];
static char current_led_char = 'L';
static const char* current_led_pattern = ".-..";
static bool led_active = false;
static bool led_symbol_on = false;
static int led_pattern_index = 0;
static uint64_t led_next_event_us = 0;

static bool motors_runtime_enabled = false;
static bool motors_initialized = false;
static bool servo_runtime_enabled = false;
static bool servo_initialized = false;
static bool timer_started = false;
static repeating_timer_t velocity_timer;
static repeating_timer_t position_timer;
static bool diag_high[DIAG_MAX_GPIO + 1] = {false};
static bool diag_pwm[DIAG_MAX_GPIO + 1] = {false};

static bool set_led_char(char led_char);

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
    return gpio == 0 || gpio == 1 || gpio == 22 || gpio == 23 || gpio == 25;
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

static uint32_t morse_symbol_on_ms(char symbol) {
    return symbol == '-' ? MORSE_DASH_MS : MORSE_DOT_MS;
}

static char* skip_spaces(char* text) {
    while (*text != 0 && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

static char* read_upper_token(char* text, char* token, int token_size) {
    text = skip_spaces(text);
    int i = 0;
    while (*text != 0 && !isspace((unsigned char)*text)) {
        if (i < token_size - 1) {
            token[i++] = (char)toupper((unsigned char)*text);
        }
        ++text;
    }
    token[i] = 0;
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
    printf("PIN_STATUS PROTECTED=0,1,22,23,25 DIAG_HIGH=");
    print_diag_gpio_list(diag_high);
    printf(" DIAG_PWM=");
    print_diag_gpio_list(diag_pwm);
    printf(" MAX_GPIO=%d\n", DIAG_MAX_GPIO);
}

static bool parse_one_int(char* text, int* a) {
    return sscanf(text, "%d", a) == 1;
}

static bool parse_two_ints(char* text, int* a, int* b) {
    return sscanf(text, "%d %d", a, b) == 2;
}

static bool parse_three_ints(char* text, int* a, int* b, int* c) {
    return sscanf(text, "%d %d %d", a, b, c) == 3;
}

static void handle_gpio_read(char* rest) {
    int gpio = -1;
    if (!parse_one_int(rest, &gpio)) {
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
    pwm_set_clkdiv(slice, (float)125E6 / (2048 * DIAG_PWM_FREQ));
    pwm_set_wrap(slice, 2047);
    pwm_set_chan_level(slice, channel, (uint16_t)(2047 * duty_percent / 100));
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
    if (!motors_runtime_enabled) {
        return timer_started;
    }
    motor[0].timer_cb();
    motor[1].timer_cb();
    return timer_started;
}

static bool timer_cb_pos(repeating_timer_t* rt) {
    if (!motors_runtime_enabled) {
        return timer_started;
    }
    motor[0].timer_cb_pos();
    motor[1].timer_cb_pos();
    return timer_started;
}

static void start_motor_timers() {
    if (timer_started) {
        return;
    }
    add_repeating_timer_ms(-10, timer_cb, NULL, &velocity_timer);
    add_repeating_timer_ms(-100, timer_cb_pos, NULL, &position_timer);
    timer_started = true;
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
    if (!motors_initialized) {
        return;
    }
    motor[0].disablePosPid();
    motor[1].disablePosPid();
    motor[0].setVel(0);
    motor[1].setVel(0);
}

static void configure_motor_gains() {
    motor[0].setVelGain(1, 0.0, 0.09);
    motor[0].setPosGain(2.5, 0.0, 0.09);
    motor[1].setVelGain(1, 0.0, 0.09);
    motor[1].setPosGain(2.5, 0.0, 0.09);
}

static bool enable_motors() {
    if (motors_pin_conflict()) {
        return false;
    }

    if (!motors_initialized) {
        gpio_set_dir(MOTOR_QENC0_GPIO, GPIO_IN);
        gpio_set_dir(MOTOR_QENC0_GPIO + 1, GPIO_IN);
        gpio_set_dir(MOTOR_QENC1_GPIO, GPIO_IN);
        gpio_set_dir(MOTOR_QENC1_GPIO + 1, GPIO_IN);
        motor[0].init();
        motor[1].init();
        configure_motor_gains();
        motors_initialized = true;
    }

    motors_runtime_enabled = true;
    stop_motors_if_enabled();
    start_motor_timers();
    return true;
}

static void disable_motors() {
    stop_motors_if_enabled();
    motors_runtime_enabled = false;
    stop_motor_timers();
}

static bool enable_servo() {
    if (servo_pin_conflict()) {
        return false;
    }
    if (!servo_initialized) {
        servo.init();
        servo_initialized = true;
    }
    servo_runtime_enabled = true;
    return true;
}

static void disable_servo() {
    servo_runtime_enabled = false;
}

static void safe_all() {
    disable_motors();
    disable_servo();
    diag_all_low();
}

static void print_status() {
    printf("FW=NORMAL UART_ID=%d TX=%d RX=%d BAUD=%d "
           "MOTORS_ENABLED=%d SERVO_ENABLED=%d TIMER_STARTED=%d "
           "PWM=%d,%d,%d,%d SERVO_GPIO=%d QENC=%d,%d LED=%c PIN_CONFLICT=",
           MOTOR_UART_ID,
           MOTOR_UART_TX_GPIO,
           MOTOR_UART_RX_GPIO,
           MOTOR_UART_BAUDRATE,
           motors_runtime_enabled ? 1 : 0,
           servo_runtime_enabled ? 1 : 0,
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

static void print_help() {
    printf("COMMANDS: PING, STATUS, STOP, SAFE, MOTOR_ENABLE, MOTOR_DISABLE, "
           "SERVO_ENABLE, SERVO_DISABLE, LED <B|U|O|A|N|M|S|L|R|P|E|I|T|X>, "
           "PIN_STATUS, GPIO_READ <gpio>, GPIO_HIGH <gpio>, GPIO_LOW <gpio>, "
           "GPIO_PULSE <gpio> <ms>, PWM_TEST <gpio> <duty> <ms>, PWM_OFF <gpio>, "
           "DIAG_ALL_LOW, motor commands\n");
    printf("MOTOR: <id> <mode> <val>\n");
}

static bool handle_text_command(char* line) {
    char command[20];
    char* rest = read_upper_token(line, command, sizeof(command));
    if (command[0] == 0) {
        return true;
    }

    if (strcmp(command, "LED") == 0) {
        rest = skip_spaces(rest);
        if (*rest == 0) {
            set_led_char('I');
            printf("ERR LED missing\n");
            fflush(stdout);
            return true;
        }

        char led_char = (char)toupper((unsigned char)*rest);
        if (!set_led_char(led_char)) {
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
        set_led_char('P');
        printf("PONG\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "STATUS") == 0) {
        print_status();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "HELP") == 0) {
        print_help();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "PIN_STATUS") == 0) {
        print_pin_status();
        fflush(stdout);
        set_led_char('D');
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
        if (!diag_runtime_available()) {
            return true;
        }
        diag_all_low();
        printf("OK DIAG_ALL_LOW\n");
        fflush(stdout);
        set_led_char('D');
        return true;
    }

    if (strcmp(command, "STOP") == 0) {
        stop_motors_if_enabled();
        set_led_char('S');
        printf("OK STOP\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SAFE") == 0) {
        safe_all();
        set_led_char('S');
        printf("OK SAFE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTOR_ENABLE") == 0) {
        if (!enable_motors()) {
            set_led_char('E');
            printf("ERR PIN_CONFLICT MOTORS\n");
            fflush(stdout);
            return true;
        }
        set_led_char('M');
        printf("OK MOTOR_ENABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "MOTOR_DISABLE") == 0) {
        disable_motors();
        set_led_char('S');
        printf("OK MOTOR_DISABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SERVO_ENABLE") == 0) {
        if (!enable_servo()) {
            set_led_char('E');
            printf("ERR PIN_CONFLICT SERVO\n");
            fflush(stdout);
            return true;
        }
        set_led_char('A');
        printf("OK SERVO_ENABLE\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "SERVO_DISABLE") == 0) {
        disable_servo();
        set_led_char('S');
        printf("OK SERVO_DISABLE\n");
        fflush(stdout);
        return true;
    }

    return false;
}

static bool readline(char* line, int line_size) {
    int i = 0;
    while (true) {
        int c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
        service_led();
        if (c_raw == PICO_ERROR_TIMEOUT) {
            continue;
        }

        char c = (char)c_raw;
        if (c == '\n') {
            break;
        }
        if (c == '\r') {
            continue;
        }
        if (i >= line_size - 1) {
            line[line_size - 1] = 0;
            while (c != '\n') {
                c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
                service_led();
                if (c_raw == PICO_ERROR_TIMEOUT) {
                    continue;
                }
                c = (char)c_raw;
            }
            return false;
        }
        line[i++] = c;
    }
    line[i] = 0;
    return true;
}

int main() {
    init_led();
    init_uart_stdio();
    set_led_char('L');
    printf("FW NORMAL SAFE BOOT\n");
    fflush(stdout);

    while (true) {
        service_led();
        if (!readline(buf, sizeof(buf))) {
            set_led_char('E');
            printf("ERR line_too_long\n");
            fflush(stdout);
            continue;
        }

        set_led_char('R');
        if (handle_text_command(buf)) {
            continue;
        }

        int id = 0;
        int mode = 0;
        double val = 0.0;
        int parsed = sscanf(buf, "%d %d %lf", &id, &mode, &val);
        if (parsed != 3) {
            set_led_char('E');
            printf("ERR parse raw=\"%s\"\n", buf);
            fflush(stdout);
            continue;
        }

        if (id < 0 || id > 2) {
            set_led_char('I');
            printf("ERR id out_of_range id=%d\n", id);
            fflush(stdout);
            continue;
        }
        if ((id == 0 || id == 1) && !motors_runtime_enabled) {
            set_led_char('E');
            printf("ERR MOTORS_DISABLED\n");
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
            set_led_char('I');
            printf("ERR mode invalid id=%d mode=%d\n", id, mode);
            fflush(stdout);
            continue;
        }

        switch (id) {
            case 0:
                if (!mode) {
                    motor[0].disablePosPid();
                    motor[0].setVel(val);
                    printf("OK motor id=0 mode=vel target=%.3f\n", val);
                } else {
                    motor[0].resetPos();
                    motor[0].setPos(val);
                    printf("OK motor id=0 mode=pos target=%.3f\n", val);
                }
                break;
            case 1:
                if (!mode) {
                    motor[1].disablePosPid();
                    motor[1].setVel(val);
                    printf("OK motor id=1 mode=vel target=%.3f\n", val);
                } else {
                    motor[1].resetPos();
                    motor[1].setPos(val);
                    printf("OK motor id=1 mode=pos target=%.3f\n", val);
                }
                break;
            case 2:
                servo.write((int)val);
                printf("OK servo id=2 mode=%d angle=%d\n", mode, (int)val);
                break;
        }
        set_led_char('T');
        fflush(stdout);
    }
}
