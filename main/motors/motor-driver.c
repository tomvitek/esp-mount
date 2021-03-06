#include "motor-driver.h"
#include <stdlib.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>

#define TAG "motor-driver"
#define PARAM_UPDATE_P 1000

const uint8_t MULTIPLIERS[] = {1, 4, 8, 16};
#define MULTIPLIERS_COUNT 4

void makeStep(motor_t m) {
    gpio_set_level(m->cfg.stepPin, 1);
    gpio_set_level(m->cfg.stepPin, 0);
    if (m->dir == 1)
        m->pos += MULTIPLIERS[m->multIdx];
    else
        m->pos -= MULTIPLIERS[m->multIdx];
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

void updateMultiplier(motor_t m, uint8_t multIdx) {
    m->multIdx = multIdx;
    switch (multIdx) {
        case 0:
            gpio_set_direction(m->cfg.cfg1Pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(m->cfg.cfg1Pin, GPIO_FLOATING);
            gpio_set_direction(m->cfg.cfg2Pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(m->cfg.cfg2Pin, GPIO_FLOATING);
            break;
        
        case 1:
            gpio_set_direction(m->cfg.cfg1Pin, GPIO_MODE_OUTPUT);
            gpio_set_direction(m->cfg.cfg2Pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(m->cfg.cfg2Pin, GPIO_FLOATING);
            gpio_set_level(m->cfg.cfg1Pin, 1);
            break;

        case 2:
            gpio_set_direction(m->cfg.cfg1Pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(m->cfg.cfg1Pin, GPIO_FLOATING);
            gpio_set_direction(m->cfg.cfg2Pin, GPIO_MODE_OUTPUT);
            gpio_set_level(m->cfg.cfg2Pin, 0);
            break;

        case 3:
            gpio_set_direction(m->cfg.cfg1Pin, GPIO_MODE_OUTPUT);
            gpio_set_direction(m->cfg.cfg2Pin, GPIO_MODE_OUTPUT);
            gpio_set_level(m->cfg.cfg1Pin, 0);
            gpio_set_level(m->cfg.cfg2Pin, 0);
            break;
    }
    ESP_LOGD(TAG, "Switched to multiplier %i", multIdx);
}

inline float getStepI(motor_t m) {
    return 1000000.0f/abs(m->v) * MULTIPLIERS[m->multIdx];
}

inline float stepIToV(float stepI, uint8_t dir) {
    return (float)1000000.0f/stepI * (dir == 0 ? -1 : 1);
}

inline float getCorrectA(motor_t m, int64_t t) {
    return 2 * ((float)(m->tPos - m->tStartPos) / (m->tTime - m->tStartTime) * 1e6 - m->tStartV)/(m->tTime - m->tStartTime) * 1e6;
}

inline float getCorrectV(motor_t m, int64_t t) {
    return (float)(m->tPos - m->pos) / ((float)(m->tTime - t) / 1e6);
}

int64_t motor_getPosOffset(motor_t m, int64_t t) {
    step_t x0 = m->tStartPos;
    step_t x1 = m->tPos;
    step_t x = m->pos;
    int64_t tdt = m->tTime - m->tStartTime; // Tracking delta time
    int64_t sdt = t - m->tStartTime; // Start delta time
    float v0 = m->tStartV;

    float correctA = ((float)(x1 - x0) / tdt * 1e6 - v0)/tdt;
    int64_t correctPos = x0 + (v0 * sdt + correctA * sdt * sdt) * 1e-6;

    if (t > m->tTime) {
        correctPos = x1;
    }
    if (t < m->tStartTime) {
        correctPos = x0;
    }
    return x - correctPos;
}

void accelV(motor_t m, float a, int64_t dt) {
    m->v += a * dt / 1000000;
    if (abs(m->v) > m->cfg.maxV) {
        m->v = abs(m->v) / m->v * m->cfg.maxV;
    }
    if ((m->v > 0.0f && m->dir == 0) || (m->v < 0.0f && m->dir == 1)) {
        updateDir(m, m->v > 0 ? 1: 0);
    }
}

inline void trackMAdjust(motor_t m, int64_t t, int64_t dt) {
    float correctV = getCorrectV(m, t);

    // Ideal speed (the average speed between startPos and endPos over endTime - startTime). 
    // Maybe it would be better to replace this with speed of future track point, but seems to work fine as it is
    float idealV = (float)(m->tPos - m->tStartPos) * 1e6 / (m->tTime - m->tStartTime) ;
    // Time (in microseconds) the motor needs to brake from current speed to idealV
    int64_t brake_time = fabs(m->v - idealV) / m->cfg.maxA * 1e6;
    // Position the motor should have in the moment it stops braking to idealV
    step_t ideal_x_in_bt = m->tStartPos + idealV * (t - m->tStartTime + brake_time) / 1e6;
    // Distance the motor needs to brake before it reaches the target speed
    step_t brake_dst = m->v * brake_time / 1e6 - 0.5e-12 * (m->v > 0 ? m->cfg.maxA : -m->cfg.maxA) * brake_time * brake_time;    
    
    if (m->v > idealV && brake_dst > (ideal_x_in_bt - m->pos)) // Braking, when the speed is too high (it would overrun the target position)
        accelV(m, -m->cfg.maxA, dt);
    else if (m->v < idealV && brake_dst < (ideal_x_in_bt - m->pos)) // Same as above, but for the opossite direction
        accelV(m, m->cfg.maxA, dt);
    else if (m->v > correctV) // Speed up if speed is too low
        accelV(m, -m->cfg.maxA, dt);
    else if (m->v < correctV) // Speed down if speed is too high
        accelV(m, m->cfg.maxA, dt);

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

void multiplierAdjust(motor_t m) {
    float stepI = getStepI(m);
    if (stepI < m->cfg.minStepI) {
        if (m->multIdx == MULTIPLIERS_COUNT - 1) {
            // Maximum multiplier reached, can do nothing
            return;
        }
        updateMultiplier(m, m->multIdx + 1);
    }
    if (m->multIdx > 0 && stepI * MULTIPLIERS[m->multIdx - 1] / MULTIPLIERS[m->multIdx] > m->cfg.minStepI) {
        updateMultiplier(m, m->multIdx - 1);
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
    motor->lastParamUpdateTime = 0;
    motor->multIdx = 0;
    return motor;
}

void motor_run(motor_t m) {
    int64_t time = esp_timer_get_time();
    bool paramsUpdated = false;
    if (m->lastParamUpdateTime + PARAM_UPDATE_P < time) {
        int64_t dt = time - m->lastParamUpdateTime;
        if (m->mode == TRACKING)
            trackMAdjust(m, time, dt);
        else if (m->mode == GOTO)
            gotoMAdjust(m, time, dt);
        m->lastParamUpdateTime = time;
        paramsUpdated = true;
    }

    if (!paramsUpdated && abs(m->v) > 0.0f && m->mode != STOP && time - m->lastStepTime > getStepI(m)) {
        //ESP_LOGD(TAG, "pos: %lli", m->pos);
        makeStep(m);
        m->lastStepTime = time;

        int64_t dt = time - m->lastParamUpdateTime;
        if (m->mode == TRACKING)
            trackMAdjust(m, time, dt);
        else if (m->mode == GOTO)
            gotoMAdjust(m, time, dt);
        m->lastParamUpdateTime = time;
        multiplierAdjust(m);
    }
}

void motor_track(motor_t m, step_t startPos, step_t targetPos, int64_t startTime, int64_t targetTime) {

    m->tStartTime = startTime;
    m->tStartPos = startPos;
    m->tPos = targetPos;
    m->tTime = targetTime;
    m->tStartV = m->v;
    m->mode = TRACKING;
}

void motor_goto(motor_t m, step_t targetPos) {
    m->tPos = targetPos;
    m->mode = GOTO;
}

void motor_stop(motor_t m, bool instant) {
    if (instant) {
        m->v = 0.0f;
        m->mode = STOP;
    }
    else {
        motor_goto(m, m->pos); // TODO: make the motor to stop and stay stopped, not to return to original position.
        m->mode = GOTO;
    }
}

void motor_destroy(motor_t motor) {
    gpio_set_direction(motor->cfg.stepPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.dirPin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.cfg1Pin, GPIO_MODE_INPUT);
    gpio_set_direction(motor->cfg.cfg2Pin, GPIO_MODE_INPUT);
    free(motor);
}