#include "motor-driver.h"
#include <stdlib.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>

#define TAG "motor-driver"
#define PARAM_UPDATE_P 1000

void makeStep(motor_t m) {
    gpio_set_level(m->cfg.stepPin, 1);
    gpio_set_level(m->cfg.stepPin, 0);
    if (m->dir == 1)
        m->pos++;
    else
        m->pos--;
}

void updateDir(motor_t m, uint8_t dir) {
    m->dir = dir;
    if (dir == 0) {
        gpio_set_level(m->cfg.dirPin, 0);
    }
    else {
        gpio_set_level(m->cfg.dirPin, 1);
    }
}

inline float vToStepI(float v) {
    return 1000000.0f/abs(v);
}

inline float stepIToV(float stepI, uint8_t dir) {
    return (float)1000000.0f/stepI * (dir == 0 ? -1 : 1);
}

int64_t motor_getPosOffset(motor_t m, int64_t t) {
    step_t x0 = m->tStartPos;
    step_t x1 = m->tPos;
    step_t x = m->pos;
    int64_t t1 = m->tTime;
    int64_t t0 = m->tStartTime;
    float v0 = m->tStartV;
    float correctA = 2 * ((float)(x1 - x0) / (t1-t0) - v0)/(t1 - t0);
    int64_t correctPos = x0 + (t - t0) * v0 + correctA * (t - t0) * (t - t0) / 2;
    if (t > t1) {
        correctPos = x1;
    }
    if (t < t0) {
        correctPos = x0;
    }
    //ESP_LOGD(TAG, "correct A: %f, correct pos: %lli, pos: %lli offset: %lli", correctA, correctPos, x, x - correctPos);
    return x - correctPos;
}

inline void accelV(motor_t m, float a, int64_t dt) {
    m->v += a * dt / 1000000;
    if (abs(m->v) > m->cfg.maxV) {
        m->v = abs(m->v) / m->v * m->cfg.maxV;
    }
    if ((m->v > 0.0f && m->dir == 0) || (m->v < 0.0f && m->dir == 1)) {
        updateDir(m, m->v > 0 ? 1: 0);
    }
}

inline void trackMAdjust(motor_t m, int64_t t, int64_t dt) {
    int64_t posOffset = motor_getPosOffset(m, t);
    float a = posOffset * m->cfg.aPosK;
    if (abs(a) > m->cfg.maxA)
        a = abs(a) / a * m->cfg.maxA;

    //if (posOffset > 0) { // TODO: simplify this to not include abs above and directly compute positive/negative a
    accelV(m, -a, dt);
    //}
    /*else if (posOffset < 0) {
        accelV(m, a, dt);
    }*/

    if (m->tTime < t)
        m->mode = GOTO;
}

inline void gotoMAdjust(motor_t m, int64_t t, int64_t dt) {
    step_t posOffset = m->pos - m->tPos;

    if (abs(m->v) <= m->cfg.gotoMinV && posOffset == 0) {
        m->v = 0.0f;
        m->mode = STOP;
        return;
    }

    step_t brakingDst = m->v * m->v / (2 * m->cfg.brakeA) + 50;
    if (posOffset > 0) {
        if (posOffset < brakingDst && m->v < 0.0f) {
            accelV(m, m->cfg.brakeA, dt);
            if (-m->v < m->cfg.gotoMinV)
                m->v = -m->cfg.gotoMinV;
            //ESP_LOGD(TAG, "Goto braking");
        }
        else {
            accelV(m, -m->cfg.brakeA, dt);
            //ESP_LOGD(TAG, "Goto braking");
        }
    }
    else {
        if (posOffset > -brakingDst && m->v > 0.0f) {
            accelV(m, -m->cfg.brakeA, dt);
            if (m->v < m->cfg.gotoMinV && m->v > 0.0f)
                m->v = m->cfg.gotoMinV;
            //ESP_LOGD(TAG, "Goto braking");
        }
        else {
            accelV(m, m->cfg.brakeA, dt);
            //ESP_LOGD(TAG, "Goto braking");
        }
    }
}

motor_t motor_create(motor_config_t cfg) {
    motor_t motor = malloc(sizeof(Motor));

    gpio_set_direction(cfg.stepPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(cfg.dirPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(cfg.cfg1Pin, GPIO_MODE_INPUT);
    gpio_set_direction(cfg.cfg2Pin, GPIO_MODE_INPUT);

    motor->cfg = cfg;
    motor->dir = 0;
    motor->pos = 0;
    motor->mode = STOP;
    motor->v = 0.0f;
    motor->stepIOffsetCounter = 0;
    motor->lastParamUpdateTime = 0;

    return motor;
}

void motor_run(motor_t m) {
    int64_t time = esp_timer_get_time();

    if (m->lastParamUpdateTime + PARAM_UPDATE_P < time) {
        int64_t dt = time - m->lastParamUpdateTime;
        if (m->mode == TRACKING)
            trackMAdjust(m, time, dt);
        else if (m->mode == GOTO)
            gotoMAdjust(m, time, dt);
        m->lastParamUpdateTime = time;
    }

    if (abs(m->v) > 0.0f && m->mode != STOP && time - m->lastStepTime > vToStepI(m->v)) {
        //ESP_LOGD(TAG, "pos: %lli", m->pos);
        makeStep(m);
        m->lastStepTime = time;

        int64_t dt = time - m->lastParamUpdateTime;
        if (m->mode == TRACKING)
            trackMAdjust(m, time, dt);
        else if (m->mode == GOTO)
            gotoMAdjust(m, time, dt);
        m->lastParamUpdateTime = time;
    }
}

void motor_track(motor_t m, step_t startPos, step_t targetPos, int64_t startTime, int64_t targetTime) {
    m->tStartTime = startTime;
    m->tStartPos = startPos;
    m->tPos = targetPos;
    m->tTime = targetTime;
    m->tStartV = m->v;
    m->mode = TRACKING;
    m->lastParamUpdateTime = esp_timer_get_time();
}

void motor_goto(motor_t m, step_t targetPos) {
    m->tPos = targetPos;
    m->mode = GOTO;
    m->lastParamUpdateTime = esp_timer_get_time();
}

void motor_destroy(motor_t motor) {
    gpio_set_direction(motor->cfg.stepPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.dirPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.cfg1Pin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.cfg2Pin, GPIO_MODE_INPUT);
    free(motor);
}