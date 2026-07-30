#include "motor.h"
#define PI 3.14159265358979323846

Motor::Motor(Pwm& pwm0, Pwm& pwm1, Encoder& enc)
    : pwm0(pwm0), pwm1(pwm1), enc(enc) {
    velpid.setGain(1, 0.0, 0.09);
    velpid.setDt(0.01);
    pospid.setGain(1.2, 0.08, 0.09);
    pospid.setDt(0.1);
    pospid.setGain(2.5, 0.0, 0.09);
    disablePosPid();
}

void Motor::init() {
    pwm0.init();
    pwm1.init();
    enc.init();
}

void Motor::setVel(float val) {
    if(val == 0){
        currentDuty = 0;
        duty(0);
        velpid.setGoal(0);
        return;
    }
    if(val > MAX_SPEED){
        val = MAX_SPEED;
    }else if(val < -MAX_SPEED){
        val = -MAX_SPEED;
    }
    velpid.setGoal(val);
}

void Motor::setPos(float target) {
    isPosPidEnabled = true;
    pospid.setGoal(target);
}

void Motor::setVelGain(float Kp, float Ki, float Kd) {
    velpid.setGain(Kp, Ki, Kd);
}
void Motor::setPosGain(float Kp, float Ki, float Kd) {
    pospid.setGain(Kp, Ki, Kd);
}

void Motor::duty(float val) {
    if (val > 0) {
        pwm0.write(val);
        pwm1.write(0);
        currentDuty = pwm0.read();
    } else {
        pwm0.write(0);
        pwm1.write(-val);
        currentDuty = -pwm1.read();
    }
}

// deg/s
float Motor::getCurrentSpeed() {
    int currentEnc = enc.get();
    // printf("Enc: %d\n", currentEnc);
    float speed = ((((float)currentEnc - (float)prevEnc) * 360.0) / 3900.0) / 0.01;
    prevEnc = currentEnc;
    for (int i = 0; i < 3; i++) {
        speeds[i + 1] = speeds[i];
    }
    speeds[0] = speed;
    float sum = 0;
    for (int i = 0; i < 1; i++) {
        sum += speeds[i];
    }
    return sum / 1;
}

float Motor::read() {
    return currentDuty;
}

void Motor::timer_cb() {
    float speed = getCurrentSpeed();
    velpid.update(speed);
    // printf("%f\n", speed);
    float a = velpid.calc();
    // printf("%f, %f\n", currentDuty, speed);
    duty(currentDuty + a / 10000);
}

void Motor::timer_cb_pos() {
    if (!isPosPidEnabled) {
        return;
    }
    float pos = enc.get() * 360.0 / 3900.0;
    pospid.update(pos);
    float a = pospid.calc();
    // printf("%f, %f\n", pos, a);
    if (std::abs(a) > 7.5) {
        setVel(a);
    }else{
        setVel(0);
        duty(0);
    }

}

void Motor::disablePosPid() {
    isPosPidEnabled = false;
}

void Motor::resetPos() {
    // setPos(0);
    disablePosPid();
    enc.set(0);
    currentPos = 0;
    prevPos = 0;
    prevEnc = 0;
}

void Motor::resetControlState() {
    velpid.reset();
    pospid.reset();
    disablePosPid();
    currentDuty = 0;
    currentPos = 0;
    prevEnc = enc.get();
    prevPos = 0;
    for (float& speed : speeds) {
        speed = 0;
    }
}
