#pragma once
#include <stdio.h>
#include "../rpico-encoder-plus/encoderBase.h"
#include "pico/stdlib.h"
#include "../rpico-pwm/pwm.h"
#include "../generic-pid/pid.h"
#include <queue>
#include <math.h>

class Motor {
   public:
    Motor(Pwm& pwm0, Pwm& pwm1, Encoder& enc);
    void init();
    void setVel(float val);
    void setPos(float target);
    void duty(float val);
    float read();
    void setVelGain(float Kp, float Ki, float Kd);
    void setPosGain(float Kp, float Ki, float Kd);
    void timer_cb();
    void timer_cb_pos();
    float getCurrentSpeed();
    void disablePosPid();
    void resetPos();
    void resetControlState();

   private:
    Pwm& pwm0;
    Pwm& pwm1;
    Encoder& enc;
    repeating_timer_t timer;
    Pid velpid, pospid;
    float currentDuty, currentPos;
    int prevEnc, prevPos;
    float speeds[10];
    bool isPosPidEnabled;
    const int MAX_SPEED = 540;
};
