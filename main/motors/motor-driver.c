#include "motor-driver.h"
#include <stdlib.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>

#define TAG "motor-driver"
#define MAX_STEP_I
void makeStep(motor_t m) {
    gpio_set_level(m->stepPin, 1);
    gpio_set_level(m->stepPin, 0);
    if (m->dir == 1)
        m->pos++;
    else
        m->pos--;
}


void updateDir(motor_t m, uint8_t dir) {
    m->dir = dir;
    if (dir == 0) {
        gpio_set_level(m->dirPin, 0);
    }
    else {
        gpio_set_level(m->dirPin, 1);
    }
}


inline float vToStepI(float v) {
    return 1000000.0f/abs(v);
}

inline float stepIToV(float stepI, uint8_t dir) {
    return (float)1000000.0f/stepI * (dir == 0 ? -1 : 1);
}

int64_t motor_getPosOffset(motor_t m, int64_t t) {
    step_t x0 = m->prevPos;
    step_t x1 = m->tPos;
    step_t x = m->pos;
    int64_t t1 = m->tTime;
    int64_t t0 = m->prevTime;
    float v0 = vToStepI(m->prevStepI);
    float correctA = 2 * ((float)(x1 - x0) / (t1-t0) - v0)/(t1 - t0);
    int64_t correctPos = x0 + (t - t0) * v0 + correctA * (t - t0) * (t - t0) / 2;
    if (t > t1) {
        correctPos = x1;
    }
    //ESP_LOGD(TAG, "correct A: %f, correct pos: %lli, pos: %lli offset: %lli", correctA, correctPos, x, x - correctPos);
    return x - correctPos;
}

inline void gotoAutoAdjust(motor_t m, int64_t t) {
    int64_t posOffset = motor_getPosOffset(m, t);
    float a = abs(posOffset) * m->aPosK;
    if (a > m->maxA)
        a = m->maxA;

    if (posOffset > 0) {
        float newV = stepIToV(m->stepI, m->dir) - a * m->stepI / 1000000;
        if (newV > m->maxV)
            newV = m->maxV;
        if (m->dir == 1 && newV < 0)
            updateDir(m, 0);
        if (newV == 0) // TODO: check for target pos reach
            m->stepI = 0;
        else
            m->stepI = vToStepI(newV);
        //ESP_LOGD(TAG, "StepI after autoAdjust: %lli, speed: %f", m->stepI, stepIToV(m->stepI, m->dir));
    }
    else if (posOffset < 0) {
        float newV = stepIToV(m->stepI, m->dir) + a * m->stepI / 1000000;
        if (newV > m->maxV)
            newV = m->maxV;
        if (m->dir == 0 && newV > 0)
            updateDir(m, 1);
        //ESP_LOGD(TAG, "newV: %f", newV);
        if (newV == 0.0f)
            m->stepI = 0;
        else
            m->stepI = vToStepI(newV);
        //ESP_LOGD(TAG, "StepI after autoAdjust: %lli, speed: %f", m->stepI, stepIToV(m->stepI, m->dir));
        
    }
    //ESP_LOGD(TAG, "Expected offset: %f", m->vExpectedOffset);
}

motor_t motor_create(int stepPin, int dirPin, int cfg1Pin, int cfg2Pin, int64_t maxA, int64_t maxV, float aPosK) {
    motor_t motor = malloc(sizeof(Motor));

    gpio_set_direction(stepPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dirPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(cfg1Pin, GPIO_MODE_INPUT);
    gpio_set_direction(cfg2Pin, GPIO_MODE_INPUT);

    motor->stepPin = stepPin;
    motor->dirPin = dirPin;
    motor->cfg1Pin = cfg1Pin;
    motor->cfg2Pin = cfg2Pin;
    motor->pos = 0;
    motor->tPos = 0;
    motor->tTime = 0;
    motor->lastStepTime = 0;
    motor->prevTime = 0;
    motor->prevPos = 0;
    motor->lastStepIChangeTime = 0;
    motor->stepI = 0.0f;
    motor->dir = 0;
    motor->maxA = maxA;
    motor->maxV = maxV;
    motor->aPosK = aPosK;
    return motor;
}

void motor_run(motor_t m) {
    //if (m->pos != m->tPos) {
        int64_t time = esp_timer_get_time();
        if ((time - m->lastStepTime > m->stepI && m->stepI > 0)) {
            //ESP_LOGD(TAG, "pos: %lli", m->pos);
            makeStep(m);
            m->lastStepTime = time;
            gotoAutoAdjust(m, time);
        }
    //}
}

void motor_goto(motor_t m, step_t targetPos, int64_t targetTime) {
    m->prevTime = esp_timer_get_time();
    m->prevPos = m->pos;
    m->tPos = targetPos;
    m->tTime = targetTime;
    if (m->stepI <= 0) {
        m->stepI = vToStepI(100);
        updateDir(m, targetPos > m->pos ? 1 : 0);
        //ESP_LOGD(TAG, "Started goto from v=0. Current stepI: %lli", m->stepI);
    }
}


void motor_destroy(motor_t motor) {
    gpio_set_direction(motor->stepPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->dirPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg1Pin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg2Pin, GPIO_MODE_INPUT);
    free(motor);
}