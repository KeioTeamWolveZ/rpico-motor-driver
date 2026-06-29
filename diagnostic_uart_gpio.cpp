#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"

#ifndef MOTOR_DIAGNOSTIC_FIRMWARE_ID
#define MOTOR_DIAGNOSTIC_FIRMWARE_ID "uart-gpio-diagnostic-no-motor"
#endif

#ifndef MOTOR_PICO_BOARD
#define MOTOR_PICO_BOARD "unknown"
#endif

#ifndef MOTOR_UART_INSTANCE_INDEX
#define MOTOR_UART_INSTANCE_INDEX 1
#endif

#ifndef MOTOR_UART_TX_GPIO
#define MOTOR_UART_TX_GPIO 20
#endif

#ifndef MOTOR_UART_RX_GPIO
#define MOTOR_UART_RX_GPIO 21
#endif

#ifndef MOTOR_UART_BAUDRATE
#define MOTOR_UART_BAUDRATE 115200
#endif

#if MOTOR_UART_INSTANCE_INDEX == 0
#define MOTOR_UART_INSTANCE uart0
#define MOTOR_UART_INSTANCE_NAME "uart0"
#elif MOTOR_UART_INSTANCE_INDEX == 1
#define MOTOR_UART_INSTANCE uart1
#define MOTOR_UART_INSTANCE_NAME "uart1"
#else
#error "MOTOR_UART_INSTANCE_INDEX must be 0 or 1"
#endif

static const uint32_t IDLE_LOG_INTERVAL_US = 2000000;
static char line_buf[255];

enum ReadLineResult {
    READLINE_OK,
    READLINE_EMPTY,
    READLINE_IDLE,
    READLINE_TOO_LONG,
};

static void debug_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    fflush(stdout);
}

static void debug_print_escaped_line(const char* prefix, const char* line) {
    printf("%s\"", prefix);
    for (const unsigned char* p = (const unsigned char*)line; *p != 0; ++p) {
        unsigned char c = *p;
        if (c == '"' || c == '\\') {
            printf("\\%c", c);
        } else if (c >= 32 && c <= 126) {
            putchar(c);
        } else {
            printf("\\x%02X", c);
        }
    }
    printf("\"\n");
    fflush(stdout);
}

static void make_gpio_input_hiz(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

static void put_known_motor_gpio_in_hiz() {
    make_gpio_input_hiz(0);  // Servo PWM in the normal firmware.
    make_gpio_input_hiz(2);  // Motor PWM candidate from normal firmware Pwm array.
    make_gpio_input_hiz(3);  // Motor PWM candidate from normal firmware Pwm array.
    make_gpio_input_hiz(6);  // Motor PWM candidate from normal firmware Pwm array.
    make_gpio_input_hiz(7);  // Motor PWM candidate from normal firmware Pwm array.
}

static void init_stdio() {
    stdio_usb_init();
    stdio_uart_init_full(
        MOTOR_UART_INSTANCE,
        MOTOR_UART_BAUDRATE,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO);
}

static void log_gpio_level(const char* name, uint pin) {
    debug_printf("DBG %s gpio=%u level=%d", name, pin, gpio_get(pin));
}

static void log_boot_config() {
    debug_printf("DBG FIRMWARE_ID=%s", MOTOR_DIAGNOSTIC_FIRMWARE_ID);
    debug_printf("DBG build_date=%s %s", __DATE__, __TIME__);
    debug_printf("DBG board=%s", MOTOR_PICO_BOARD);
    debug_printf("DBG uart_instance=%s", MOTOR_UART_INSTANCE_NAME);
    debug_printf("DBG uart_baud=%d", MOTOR_UART_BAUDRATE);
    debug_printf("DBG uart_tx_gpio=%d", MOTOR_UART_TX_GPIO);
    debug_printf("DBG uart_rx_gpio=%d", MOTOR_UART_RX_GPIO);
    debug_printf("DBG no motor init");
    debug_printf("DBG no servo init");
    debug_printf("DBG gpio tx level=%d", gpio_get(MOTOR_UART_TX_GPIO));
    debug_printf("DBG gpio rx level=%d", gpio_get(MOTOR_UART_RX_GPIO));
    debug_printf("DBG gpio20 level=%d", gpio_get(20));
    debug_printf("DBG gpio21 level=%d", gpio_get(21));
    log_gpio_level("motor_candidate_gpio0", 0);
    log_gpio_level("motor_candidate_gpio2", 2);
    log_gpio_level("motor_candidate_gpio3", 3);
    log_gpio_level("motor_candidate_gpio6", 6);
    log_gpio_level("motor_candidate_gpio7", 7);
}

static ReadLineResult readline(char* buf, size_t buf_size) {
    size_t i = 0;
    while (true) {
        int c_raw = getchar_timeout_us(IDLE_LOG_INTERVAL_US);
        if (c_raw == PICO_ERROR_TIMEOUT) {
            if (i == 0) {
                return READLINE_IDLE;
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
        if (i >= buf_size - 1) {
            buf[buf_size - 1] = 0;
            while (c != '\n') {
                c_raw = getchar_timeout_us(IDLE_LOG_INTERVAL_US);
                if (c_raw == PICO_ERROR_TIMEOUT) {
                    break;
                }
                c = (char)c_raw;
            }
            return READLINE_TOO_LONG;
        }
        buf[i++] = c;
    }
    buf[i] = 0;
    if (i == 0) {
        return READLINE_EMPTY;
    }
    return READLINE_OK;
}

int main() {
    put_known_motor_gpio_in_hiz();
    init_stdio();

    debug_printf("DBG boot");
    log_boot_config();
    debug_printf("DBG loop start");

    while (true) {
        ReadLineResult line_result = readline(line_buf, sizeof(line_buf));
        if (line_result == READLINE_IDLE) {
            debug_printf("DBG idle waiting");
            continue;
        }
        if (line_result == READLINE_EMPTY) {
            continue;
        }
        if (line_result == READLINE_TOO_LONG) {
            debug_printf("ERR line_too_long");
            continue;
        }

        debug_print_escaped_line("DBG rx raw=", line_buf);
        debug_printf("DBG rx len=%u", (unsigned int)strlen(line_buf));

        int id = 0;
        int mode = 0;
        double val = 0.0;
        int parsed = sscanf(line_buf, "%d %d %lf", &id, &mode, &val);
        debug_printf("DBG parse n=%d id=%d mode=%d val=%.3f", parsed, id, mode, val);
        if (parsed != 3) {
            debug_print_escaped_line("ERR parse raw=", line_buf);
            continue;
        }

        debug_printf("DBG diagnostic only: motor command ignored");
    }
}
