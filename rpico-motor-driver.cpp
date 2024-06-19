#include <stdio.h>
#include <cstdlib>
#include "../lib/rpico-pwm/pwm.h"
#include "hardware/pio.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "../lib/rpico-motor/motor.h"
#include "../lib/rpico-encoder-plus/qenc.h"
// #include "encoder.pio.h"


Pwm pwm[4] = {Pwm(6, 10000), Pwm(7, 10000), Pwm(2, 10000), Pwm(3, 10000)};
Qenc enc[2] = {Qenc(26), Qenc(28)};

Motor motor[2] = {Motor(pwm[0], pwm[3], enc[0]) , Motor(pwm[2], pwm[1], enc[1])};

char buf[255];

void readline(char* buf) {
    int i = 0;
    while (1) {
        char c = getchar();
        if (c == '\n') {
            break;
        }
        buf[i++] = c;
    }
    buf[i] = 0;
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
    stdio_init_all();
    motor[0].init();
    motor[1].init();
    // stdio_set_chars_available_callback(uart_cb, nullptr);
}

int main() {
    setup();
    while (true) {
        readline(buf);
        double duty[2]={0,0};
        sscanf(buf, "%lf %lf", &duty[0], &duty[1]);
        motor[0].duty(duty[0]);
        // motor[1].duty(-0.5);
        motor[1].duty(duty[1]);
        printf("motor[0]: %f, motor[1]: %f", duty[0], duty[1]);
    }
}
