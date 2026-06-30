#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "hardware/uart.h"
#include "pico/error.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"

#ifndef MOTOR_UART_ID
#define MOTOR_UART_ID 1
#endif

#ifndef MOTOR_UART_BAUDRATE
#define MOTOR_UART_BAUDRATE 115200
#endif

#ifndef MOTOR_UART_TX_GPIO
#define MOTOR_UART_TX_GPIO 6
#endif

#ifndef MOTOR_UART_RX_GPIO
#define MOTOR_UART_RX_GPIO 7
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
static const uint32_t UART_POLL_US = 10000;

static char line_buf[96];
static int line_len = 0;
static char current_led_char = 'D';
static const char* current_led_pattern = "-..";
static bool led_active = false;
static bool led_symbol_on = false;
static int led_pattern_index = 0;
static uint64_t led_next_event_us = 0;

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
        case 'D':
            return "-..";
        case 'P':
            return ".--.";
        case 'R':
            return ".-.";
        case 'E':
            return ".";
        case 'I':
            return "..";
        case 'L':
            return ".-..";
        case 'A':
            return ".-";
        case 'N':
            return "-.";
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

static void print_status() {
    printf("DIAG=UART_D6D7 UART_ID=%d TX=%d RX=%d BAUD=%d LED=%c\n",
           MOTOR_UART_ID,
           MOTOR_UART_TX_GPIO,
           MOTOR_UART_RX_GPIO,
           MOTOR_UART_BAUDRATE,
           current_led_char);
}

static void print_help() {
    printf("COMMANDS: PING, STATUS, LED <D|P|R|E|I|L|A|N>, HELP\n");
}

static void handle_line(char* line) {
    char command[16];
    char* rest = read_upper_token(line, command, sizeof(command));
    if (command[0] == 0) {
        return;
    }

    char previous_led_char = current_led_char;
    set_led_char('R');

    if (strcmp(command, "PING") == 0) {
        printf("PONG\n");
        fflush(stdout);
        set_led_char('P');
        return;
    }

    if (strcmp(command, "STATUS") == 0) {
        set_led_char(previous_led_char);
        print_status();
        fflush(stdout);
        return;
    }

    if (strcmp(command, "LED") == 0) {
        rest = skip_spaces(rest);
        if (*rest == 0) {
            printf("ERR LED missing\n");
            fflush(stdout);
            set_led_char('E');
            return;
        }

        char led_char = (char)toupper((unsigned char)*rest);
        if (!set_led_char(led_char)) {
            printf("ERR LED unsupported char=%c\n", led_char);
            fflush(stdout);
            set_led_char('I');
            return;
        }

        printf("OK LED %c\n", current_led_char);
        fflush(stdout);
        return;
    }

    if (strcmp(command, "HELP") == 0) {
        set_led_char(previous_led_char);
        print_help();
        fflush(stdout);
        return;
    }

    printf("ERR invalid command\n");
    fflush(stdout);
    set_led_char('I');
}

static void service_uart() {
    int c_raw = getchar_timeout_us(UART_POLL_US);
    if (c_raw == PICO_ERROR_TIMEOUT) {
        return;
    }

    char c = (char)c_raw;
    if (c == '\r') {
        return;
    }

    if (c == '\n') {
        line_buf[line_len] = 0;
        handle_line(line_buf);
        line_len = 0;
        return;
    }

    if (line_len >= (int)sizeof(line_buf) - 1) {
        line_len = 0;
        printf("ERR line too long\n");
        fflush(stdout);
        set_led_char('E');
        return;
    }

    line_buf[line_len++] = c;
}

int main() {
    init_led();
    init_uart_stdio();
    set_led_char('D');

    while (true) {
        service_led();
        service_uart();
    }
}
