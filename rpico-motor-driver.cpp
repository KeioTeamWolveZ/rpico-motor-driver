#include <stdio.h>
#include <cstdlib>
#include "../lib/rpico-encoder-plus/qenc.h"
#include "../lib/rpico-motor/motor.h"
#include "../lib/rpico-pwm/pwm.h"
#include "../lib/rpico-servo/servo.h"
#include "hardware/pio.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
// #include "encoder.pio.h"

Pwm pwm[4] = {Pwm(6, 50000), Pwm(7, 50000), Pwm(2, 50000), Pwm(3, 50000)};
Servo servo(0);
Qenc enc[2] = {Qenc(26), Qenc(28)};

Motor motor[2] = {Motor(pwm[3], pwm[0], enc[0]), Motor(pwm[1], pwm[2], enc[1])};

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
    gpio_set_dir(26, GPIO_IN);
    gpio_set_dir(27, GPIO_IN);
    gpio_set_dir(28, GPIO_IN);
    gpio_set_dir(29, GPIO_IN);
    motor[0].init();
    motor[1].init();
    servo.init();
    initTimer();
}

int main() {
    setup();
    while (true) {
        readline(buf);
        int id;
        int mode;
        double val;
        sscanf(buf, "%d %d %lf", &id, &mode, &val);
        printf("id: %d mode: %d val: %f\n", id, mode, val);
        switch (id) {
            case 0:
                if (!mode) {
                    motor[0].disablePosPid();
                    motor[0].setVel(val);
                } else {
                    motor[0].resetPos();
                    motor[0].setPos(val);
                }
                break;
            case 1:
                if (!mode) {
                    motor[1].disablePosPid();
                    motor[1].setVel(val);
                } else {
                    motor[1].resetPos();
                    motor[1].setPos(val);
                }
                break;
            case 2:
                servo.write((int)val);
                break;
        }
    }
}
