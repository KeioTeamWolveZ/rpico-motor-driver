#include <ctype.h>
#include <stdio.h>
#include <string.h>
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

Pwm pwm[4] = {Pwm(6, 50000), Pwm(7, 50000), Pwm(2, 50000), Pwm(3, 50000)};
Servo servo(0);
Qenc enc[2] = {Qenc(26), Qenc(28)};

Motor motor[2] = {Motor(pwm[3], pwm[0], enc[0]), Motor(pwm[1], pwm[2], enc[1])};

char buf[255];
static char current_led_char = 'L';
static const char* current_led_pattern = ".-..";
static bool led_active = false;
static bool led_symbol_on = false;
static int led_pattern_index = 0;
static uint64_t led_next_event_us = 0;

void led_write(bool on) {
    gpio_put(LED_PIN, on ? LED_ON : LED_OFF);
}

void init_led() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led_write(false);
}

const char* morse_pattern_for(char led_char) {
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

bool set_led_char(char led_char) {
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

char uart_pin_led_char() {
    if (MOTOR_UART_TX_GPIO == 20 && MOTOR_UART_RX_GPIO == 21) {
        return 'A';
    }
    if (MOTOR_UART_TX_GPIO == 21 && MOTOR_UART_RX_GPIO == 20) {
        return 'N';
    }
    return 'E';
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

char* skip_spaces(char* text) {
    while (*text != 0 && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

char* read_upper_token(char* text, char* token, int token_size) {
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

void print_uart_status() {
    printf("UART_ID=%d TX=%d RX=%d BAUD=%d LED=%c\n",
           MOTOR_UART_ID,
           MOTOR_UART_TX_GPIO,
           MOTOR_UART_RX_GPIO,
           MOTOR_UART_BAUDRATE,
           current_led_char);
}

void print_help() {
    printf("COMMANDS: LED <B|U|O|A|N|M|S|L|R|P|E|I|T|X>, PING, STATUS, STOP, HELP\n");
    printf("MOTOR: <id> <mode> <val>\n");
}

bool handle_diag_command(char* line) {
    char command[16];
    char* rest = read_upper_token(line, command, sizeof(command));
    if (command[0] == 0) {
        return false;
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
        set_led_char('P');
        print_uart_status();
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "STOP") == 0) {
        // No existing project-level safe stop helper is present in this old base.
        set_led_char('I');
        printf("ERR STOP unsupported\n");
        fflush(stdout);
        return true;
    }

    if (strcmp(command, "HELP") == 0) {
        set_led_char('P');
        print_help();
        fflush(stdout);
        return true;
    }

    return false;
}

void service_led() {
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

bool readline(char* buf, int buf_size) {
    int i = 0;
    while (1) {
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
        if (i >= buf_size - 1) {
            buf[buf_size - 1] = 0;
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
    set_led_char('B');
    set_led_char('U');
    init_uart_stdio();
    set_led_char('O');
    printf("DBG boot\n");
    set_led_char('M');
    setup();
    set_led_char(uart_pin_led_char());
    printf("DBG loop start\n");
    while (true) {
        service_led();
        if (!readline(buf, sizeof(buf))) {
            set_led_char('E');
            printf("ERR line_too_long\n");
            fflush(stdout);
            continue;
        }
        set_led_char('R');
        printf("DBG rx raw=\"%s\"\n", buf);
        if (handle_diag_command(buf)) {
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
        set_led_char('P');
        printf("DBG parsed id=%d mode=%d val=%.3f\n", id, mode, val);

        if (id < 0 || id > 2) {
            set_led_char('I');
            printf("ERR id out_of_range id=%d\n", id);
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
