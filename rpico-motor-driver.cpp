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

bool readline(char* buf, int buf_size) {
    int i = 0;
    while (1) {
        char c = getchar();
        if (c == '\n') {
            break;
        }
        if (c == '\r') {
            continue;
        }
        if (i >= buf_size - 1) {
            buf[buf_size - 1] = 0;
            while (c != '\n') {
                c = getchar();
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
    stdio_init_all();
    printf("DBG boot\n");
    setup();
    printf("DBG loop start\n");
    while (true) {
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
