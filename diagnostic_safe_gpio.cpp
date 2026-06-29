#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#ifndef MOTOR_SAFE_DIAGNOSTIC_FIRMWARE_ID
#define MOTOR_SAFE_DIAGNOSTIC_FIRMWARE_ID "uart-gpio-diagnostic-safe-output"
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

struct SafeGpio {
    uint pin;
    const char* reason;
};

static const SafeGpio MOTOR_SAFE_GPIOS[] = {
    {2, "motor PWM candidate, active-high assumed"},
    {3, "motor PWM candidate, active-high assumed"},
    {6, "motor PWM candidate, active-high assumed"},
    {7, "motor PWM candidate, active-high assumed"},
};

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

static void apply_motor_safe_gpio_states() {
    for (size_t i = 0; i < sizeof(MOTOR_SAFE_GPIOS) / sizeof(MOTOR_SAFE_GPIOS[0]); ++i) {
        uint pin = MOTOR_SAFE_GPIOS[i].pin;
        // GPIO2/3/6/7 are Pwm candidates in the existing normal firmware.
        // OUT Low is a non-drive candidate only if the H-bridge inputs are active-high.
        // Confirm Low-safe behavior against the schematic and motor driver datasheet.
        // Full safety before main() still requires hardware pulldowns or equivalent defaults.
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }
}

static void init_stdio() {
    stdio_usb_init();
    stdio_uart_init_full(
        MOTOR_UART_INSTANCE,
        MOTOR_UART_BAUDRATE,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO);
}

static void log_gpio_level(uint pin) {
    debug_printf("DBG gpio%u level=%d", pin, gpio_get(pin));
}

static void log_boot_config() {
    debug_printf("DBG FIRMWARE_ID=%s", MOTOR_SAFE_DIAGNOSTIC_FIRMWARE_ID);
    debug_printf("DBG build_date=%s %s", __DATE__, __TIME__);
    debug_printf("DBG board=%s", MOTOR_PICO_BOARD);
    debug_printf("DBG uart_instance=%s", MOTOR_UART_INSTANCE_NAME);
    debug_printf("DBG uart_baud=%d", MOTOR_UART_BAUDRATE);
    debug_printf("DBG uart_tx_gpio=%d", MOTOR_UART_TX_GPIO);
    debug_printf("DBG uart_rx_gpio=%d", MOTOR_UART_RX_GPIO);
    debug_printf("DBG no Pwm objects");
    debug_printf("DBG no Motor objects");
    debug_printf("DBG no Servo objects");
    debug_printf("DBG no Qenc objects");
    debug_printf("DBG applying motor safe gpio states");
    for (size_t i = 0; i < sizeof(MOTOR_SAFE_GPIOS) / sizeof(MOTOR_SAFE_GPIOS[0]); ++i) {
        debug_printf(
            "DBG safe gpio pin=%u mode=OUT value=0 reason=%s",
            MOTOR_SAFE_GPIOS[i].pin,
            MOTOR_SAFE_GPIOS[i].reason);
    }
    log_gpio_level(2);
    log_gpio_level(3);
    log_gpio_level(6);
    log_gpio_level(7);
    log_gpio_level(20);
    log_gpio_level(21);
    debug_printf("DBG no motor init");
    debug_printf("DBG no servo init");
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
    apply_motor_safe_gpio_states();
    init_stdio();

    log_boot_config();
    debug_printf("DBG boot");
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

        debug_print_escaped_line("DBG rx line=", line_buf);

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
