#include <stdio.h>
#include <cstdlib>
#include "../lib/rpico-encoder-plus/qenc.h"
#include "../lib/rpico-motor/motor.h"
#include "../lib/rpico-pwm/pwm.h"
#include "../lib/rpico-servo/servo.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"
// #include "encoder.pio.h"

#ifndef MOTOR_UART_ID
#define MOTOR_UART_ID 1
#endif

#ifndef MOTOR_UART_BAUDRATE
#define MOTOR_UART_BAUDRATE 115200
#endif

#ifndef MOTOR_UART_TX_GPIO
#define MOTOR_UART_TX_GPIO 20
#endif

#ifndef MOTOR_UART_RX_GPIO
#define MOTOR_UART_RX_GPIO 21
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
// Pico SDK seeed_xiao_rp2350.h defines the XIAO RP2350 user LED as GPIO25.
static const uint LED_PIN = 25;
#endif

// XIAO user LEDs are commonly active-low. Flip these two constants if the
// mounted board lights the LED with a high output instead.
static const bool LED_ON = false;
static const bool LED_OFF = true;
static const uint32_t MORSE_DOT_MS = 150;
static const uint32_t MORSE_DASH_MS = 500;
static const uint32_t MORSE_SYMBOL_GAP_MS = 150;
static const uint32_t MORSE_CHAR_GAP_MS = 1000;
static const uint32_t LOOP_ALIVE_SERVICE_US = 10000;
static const uint64_t LOOP_ALIVE_REPEAT_US = 2000000;

Pwm pwm[4] = {Pwm(6, 50000), Pwm(7, 50000), Pwm(2, 50000), Pwm(3, 50000)};
Servo servo(0);
Qenc enc[2] = {Qenc(26), Qenc(28)};

Motor motor[2] = {Motor(pwm[3], pwm[0], enc[0]), Motor(pwm[1], pwm[2], enc[1])};

char buf[255];

void led_write(bool on) {
    gpio_put(LED_PIN, on ? LED_ON : LED_OFF);
}

void init_led() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led_write(false);
}

void blink_dot() {
    led_write(true);
    sleep_ms(MORSE_DOT_MS);
    led_write(false);
}

void blink_dash() {
    led_write(true);
    sleep_ms(MORSE_DASH_MS);
    led_write(false);
}

void blink_morse(const char* pattern) {
    for (int i = 0; pattern[i] != 0; ++i) {
        if (pattern[i] == '.') {
            blink_dot();
        } else if (pattern[i] == '-') {
            blink_dash();
        }

        if (pattern[i + 1] != 0) {
            sleep_ms(MORSE_SYMBOL_GAP_MS);
        }
    }
    sleep_ms(MORSE_CHAR_GAP_MS);
}

void blink_error_forever() {
    while (true) {
        blink_dot();
        sleep_ms(MORSE_SYMBOL_GAP_MS);
    }
}

void blink_uart_pin_pattern() {
    if (MOTOR_UART_TX_GPIO == 20 && MOTOR_UART_RX_GPIO == 21) {
        blink_morse(".-");
    } else if (MOTOR_UART_TX_GPIO == 21 && MOTOR_UART_RX_GPIO == 20) {
        blink_morse("-.");
    } else {
        // Undefined UART pin pattern: show E three times so it is visible.
        blink_morse(".");
        blink_morse(".");
        blink_morse(".");
    }
}

void init_uart_stdio() {
    stdio_uart_init_full(
        MOTOR_UART_INSTANCE,
        MOTOR_UART_BAUDRATE,
        MOTOR_UART_TX_GPIO,
        MOTOR_UART_RX_GPIO);
}

uint32_t morse_symbol_on_ms(char symbol) {
    return symbol == '-' ? MORSE_DASH_MS : MORSE_DOT_MS;
}

void service_loop_alive_led() {
    static const char pattern[] = ".-..";
    static bool enabled = false;
    static bool initialized = false;
    static bool active = false;
    static bool symbol_on = false;
    static int index = 0;
    static uint64_t cycle_start_us = 0;
    static uint64_t next_event_us = 0;

    uint64_t now_us = time_us_64();
    if (!initialized) {
        initialized = true;
        enabled = true;
        cycle_start_us = now_us;
        next_event_us = now_us;
    }

    if (!enabled || now_us < next_event_us) {
        return;
    }

    if (!active) {
        active = true;
        symbol_on = true;
        index = 0;
        cycle_start_us = now_us;
        led_write(true);
        next_event_us = now_us + morse_symbol_on_ms(pattern[index]) * 1000;
        return;
    }

    if (symbol_on) {
        led_write(false);
        symbol_on = false;
        if (pattern[index + 1] == 0) {
            next_event_us = now_us + MORSE_CHAR_GAP_MS * 1000;
        } else {
            next_event_us = now_us + MORSE_SYMBOL_GAP_MS * 1000;
        }
        return;
    }

    if (pattern[index + 1] == 0) {
        active = false;
        index = 0;
        uint64_t next_cycle_us = cycle_start_us + LOOP_ALIVE_REPEAT_US;
        next_event_us = now_us < next_cycle_us ? next_cycle_us : now_us;
        return;
    }

    ++index;
    symbol_on = true;
    led_write(true);
    next_event_us = now_us + morse_symbol_on_ms(pattern[index]) * 1000;
}

bool readline(char* buf, int buf_size) {
    int i = 0;
    while (1) {
        int c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
        service_loop_alive_led();
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
        if (i >= buf_size - 1) {
            buf[buf_size - 1] = 0;
            while (c != '\n') {
                c_raw = getchar_timeout_us(LOOP_ALIVE_SERVICE_US);
                service_loop_alive_led();
                if (c_raw == PICO_ERROR_TIMEOUT) {
                    continue;
                }
                c = (char)c_raw;
            }
            return false;
        }
        buf[i++] = c;
    }
    buf[i] = 0;
    return true;
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

void setup() {
    printf("DBG setup start\n");
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
    initTimer();
    printf("DBG setup done\n");
}

int main() {
    init_led();
    // These diagnostic delays run before motor setup so the old motor/servo
    // initialization order stays intact while boot progress is visible.
    blink_morse("-...");
    blink_morse("..-");
    init_uart_stdio();
    blink_morse("---");
    blink_uart_pin_pattern();
    printf("DBG boot\n");
    blink_morse("--");
    setup();
    blink_morse("...");
    printf("DBG loop start\n");
    while (true) {
        service_loop_alive_led();
        if (!readline(buf, sizeof(buf))) {
            printf("ERR line_too_long\n");
            continue;
        }
        printf("DBG rx raw=\"%s\"\n", buf);

        int id = 0;
        int mode = 0;
        double val = 0.0;
        int parsed = sscanf(buf, "%d %d %lf", &id, &mode, &val);
        if (parsed != 3) {
            printf("ERR parse raw=\"%s\"\n", buf);
            continue;
        }
        printf("DBG parsed id=%d mode=%d val=%.3f\n", id, mode, val);

        if (id < 0 || id > 2) {
            printf("ERR id out_of_range id=%d\n", id);
            continue;
        }
        if ((id == 0 || id == 1) && mode != 0 && mode != 1) {
            printf("ERR mode invalid id=%d mode=%d\n", id, mode);
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
    }
}
