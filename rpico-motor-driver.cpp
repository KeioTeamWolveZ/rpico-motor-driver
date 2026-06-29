#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lib/rpico-encoder-plus/qenc.h"
#include "lib/rpico-motor/motor.h"
#include "lib/rpico-pwm/pwm.h"
#include "lib/rpico-servo/servo.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
// #include "encoder.pio.h"

#ifndef MOTOR_FIRMWARE_VERSION
#define MOTOR_FIRMWARE_VERSION "dev"
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

Pwm pwm[4] = {Pwm(6, 50000), Pwm(7, 50000), Pwm(2, 50000), Pwm(3, 50000)};
Servo servo(0);
Qenc enc[2] = {Qenc(26), Qenc(28)};

Motor motor[2] = {Motor(pwm[3], pwm[0], enc[0]), Motor(pwm[1], pwm[2], enc[1])};

char buf[255];

enum ReadLineResult {
    READLINE_OK,
    READLINE_EMPTY,
    READLINE_IDLE,
    READLINE_TOO_LONG,
};

void debug_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
    fflush(stdout);
}

void debug_print_escaped_line(const char* prefix, const char* line) {
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

void init_uart_stdio() {
    stdio_uart_init_full(
        MOTOR_UART_INSTANCE,
        MOTOR_UART_BAUDRATE,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO);
}

void log_uart_config() {
    debug_printf("DBG firmware=%s", MOTOR_FIRMWARE_VERSION);
    debug_printf("DBG build_date=%s %s", __DATE__, __TIME__);
    debug_printf("DBG board=%s", MOTOR_PICO_BOARD);
    debug_printf("DBG uart_instance=%s", MOTOR_UART_INSTANCE_NAME);
    debug_printf("DBG uart_baud=%d", MOTOR_UART_BAUDRATE);
    debug_printf("DBG uart_tx_gpio=%d", MOTOR_UART_TX_GPIO);
    debug_printf("DBG uart_rx_gpio=%d", MOTOR_UART_RX_GPIO);
    debug_printf("DBG gpio tx level=%d", gpio_get(MOTOR_UART_TX_GPIO));
    debug_printf("DBG gpio rx level=%d", gpio_get(MOTOR_UART_RX_GPIO));
}

ReadLineResult readline(char* line_buf, size_t buf_size) {
    size_t i = 0;
    while (1) {
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
            line_buf[buf_size - 1] = 0;
            while (c != '\n') {
                c_raw = getchar_timeout_us(IDLE_LOG_INTERVAL_US);
                if (c_raw == PICO_ERROR_TIMEOUT) {
                    break;
                }
                c = (char)c_raw;
            }
            return READLINE_TOO_LONG;
        }
        line_buf[i++] = c;
    }
    line_buf[i] = 0;
    if (i == 0) {
        return READLINE_EMPTY;
    }
    return READLINE_OK;
}

bool timer_cb(repeating_timer_t* rt) {
    motor[0].timer_cb();
    motor[1].timer_cb();
    return true;
}

bool timer_cb_pos(repeating_timer_t* rt) {
    motor[0].timer_cb_pos();
    motor[1].timer_cb_pos();
    return true;
}

void initTimer() {
    static repeating_timer_t timer;
    static repeating_timer_t timer1;
    add_repeating_timer_ms(-10, timer_cb, NULL, &timer);
    add_repeating_timer_ms(-100, timer_cb_pos, NULL, &timer1);
}

void stop_all_motors() {
    for (int i = 0; i < 2; ++i) {
        motor[i].disablePosPid();
        motor[i].setVel(0.0f);
        motor[i].duty(0.0f);
    }
}

void setup() {
    debug_printf("DBG setup start");
    gpio_set_dir(26, GPIO_IN);
    gpio_set_dir(27, GPIO_IN);
    gpio_set_dir(28, GPIO_IN);
    gpio_set_dir(29, GPIO_IN);
    motor[0].init();
    motor[1].init();

    motor[0].setVelGain(1, 0.0, 0.09);
    motor[0].setPosGain(2.5, 0.0, 0.09);
    motor[1].setVelGain(1, 0.0, 0.09);
    motor[1].setPosGain(2.5, 0.0, 0.09);

    servo.init();
    stop_all_motors();
    initTimer();
    debug_printf("DBG setup done");
}

int main() {
    init_uart_stdio();
    debug_printf("DBG boot");
    log_uart_config();
    setup();
    debug_printf("DBG loop start");
    bool waiting_log_printed = false;
    while (true) {
        if (!waiting_log_printed) {
            debug_printf("DBG waiting for line");
            waiting_log_printed = true;
        }

        ReadLineResult line_result = readline(buf, sizeof(buf));
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
        waiting_log_printed = false;

        debug_print_escaped_line("DBG rx raw=", buf);
        debug_printf("DBG rx len=%u", (unsigned int)strlen(buf));

        int id = 0;
        int mode = 0;
        double val = 0.0;
        int parsed = sscanf(buf, "%d %d %lf", &id, &mode, &val);
        debug_printf("DBG parse n=%d id=%d mode=%d val=%.3f", parsed, id, mode, val);
        if (parsed != 3) {
            debug_print_escaped_line("ERR parse raw=", buf);
            continue;
        }

        debug_printf("DBG dispatch id=%d mode=%d val=%.3f", id, mode, val);

        switch (id) {
            case 0:
            case 1:
                debug_printf("DBG motor id=%d selected", id);
                if (mode == 0) {
                    debug_printf("DBG motor id=%d mode=velocity target=%.3f", id, val);
                    motor[id].disablePosPid();
                    motor[id].setVel((float)val);
                    debug_printf("OK motor id=%d mode=vel target=%.3f", id, val);
                } else if (mode == 1) {
                    debug_printf("DBG motor id=%d mode=position target=%.3f", id, val);
                    motor[id].resetPos();
                    motor[id].setPos((float)val);
                    debug_printf("OK motor id=%d mode=pos target=%.3f", id, val);
                } else {
                    debug_printf(
                        "ERR motor unsupported_mode id=%d mode=%d val=%.3f",
                        id,
                        mode,
                        val);
                }
                break;
            case 2:
                debug_printf("DBG servo selected");
                debug_printf("DBG servo mode=%d target=%.3f", mode, val);
                servo.write((int)val);
                debug_printf("OK servo target=%.3f", val);
                break;
            default:
                debug_printf("ERR unsupported_id id=%d mode=%d val=%.3f", id, mode, val);
                break;
        }
    }
}
