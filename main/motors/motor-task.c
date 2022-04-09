#include "motor-task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "../settings.h"
#include <esp_timer.h>
#include "motor-driver.h"
#include <math.h>
#define TAG "motor-task"

//#define MEASURE_CYCLE_T

bool tracking = false;
motor_t m1;
motor_t m2;
TrackPoint currentTrackPoint;
QueueHandle_t motorCmdQueue;

void beginTracking(uint64_t time) {
    bool trackPointAcquired = mount_pullTrackPoint(&currentTrackPoint);

    if (!trackPointAcquired) {
        ESP_LOGW(TAG, "Requested tracking start, but no tracking point found");
        motor_stop(m1, false);
        motor_stop(m2, false);
        tracking = false;
    }
    else {
        motor_goto(m1, currentTrackPoint.ax1);
        motor_goto(m2, currentTrackPoint.ax2);
        tracking = true;
    }
}

inline int64_t timeToESPTime(uint64_t time, int64_t currentESPTime, uint64_t currentTime) {
    return currentESPTime + (time - currentTime) * 1000;

}

void updateTracking(uint64_t time) {
    int64_t espTime = esp_timer_get_time();
    if (currentTrackPoint.time < time) {
        TrackPoint newTrackPoint;
        bool trackPointAcquired = mount_pullTrackPoint(&newTrackPoint);
        if (!trackPointAcquired) {
            motor_goto(m1, currentTrackPoint.ax1);
            motor_goto(m2, currentTrackPoint.ax2);
            tracking = false;
            return;
        }

        int64_t currentTrackEspTime = timeToESPTime(currentTrackPoint.time, espTime, time);
        int64_t newTrackEspTime = timeToESPTime(newTrackPoint.time, espTime, time);
        motor_track(m1, currentTrackPoint.ax1, newTrackPoint.ax1, currentTrackEspTime, newTrackEspTime);
        motor_track(m2, currentTrackPoint.ax2, newTrackPoint.ax2, currentTrackEspTime, newTrackEspTime);
        currentTrackPoint = newTrackPoint;
    }
}

void processQueue(uint64_t time) {
    MotorCmd cmd;
    if (xQueueReceive(motorCmdQueue, &cmd, 0) == pdPASS) {
        if (cmd.type == CMD_POSITION_UPDATE) {
            m1->pos = cmd.data.pos.ax1;
            m2->pos = cmd.data.pos.ax2;
            ESP_LOGD(TAG, "Position updated");
        }
        else if (cmd.type == CMD_GOTO) {
            motor_goto(m1, cmd.data.pos.ax1);
            motor_goto(m2, cmd.data.pos.ax2);
            tracking = false;
        }
        else if (cmd.type == CMD_STOP) {
            motor_stop(m1, cmd.data.instantStop);
            motor_stop(m2, cmd.data.instantStop);
            tracking = false;
        }
        else if (cmd.type == CMD_TRACK_BEGIN) {
            beginTracking(time);
        }
        else if (cmd.type == CMD_TRACK_STOP) {
            motor_stop(m1, false);
            motor_stop(m2, false);
            tracking = false;
        }
    }
}

void updateState() {
    MountStatus status;
    if (tracking)
        status = MOUNT_STATUS_TRACKING;
    else if (m1->mode == GOTO || m2->mode == GOTO)
        status = MOUNT_STATUS_GOTO;
    else if (m1->mode == BRAKING || m2->mode == BRAKING)
        status = MOUNT_STATUS_BRAKING;
    else
        status = MOUNT_STATUS_STOPPED;
    
    mount_setState(status, m1->pos, m2->pos);
}

void motor_task(void *args) {
    motorCmdQueue = args;

    motor_config_t m1Cfg = {
        .stepPin = MOTOR_DEC_STEP_PIN,
        .dirPin = MOTOR_DEC_DIR_PIN,
        .cfg1Pin = MOTOR_DEC_CFG1_PIN,
        .cfg2Pin = MOTOR_DEC_CFG2_PIN,
        .maxA = MOTOR_MAX_A,
        .maxV = MOTOR_MAX_V,
        .brakeA = MOTOR_BRAKE_A,
        .aPosK = MOTOR_A_POS_K,
        .gotoMinV = MOTOR_GOTO_MIN_V,
        .minStepI = MOTOR_MIN_STEP_I_MICROS
    };
    m1 = motor_create(m1Cfg);

    motor_config_t m2Cfg = {
        .stepPin = MOTOR_RA_STEP_PIN,
        .dirPin = MOTOR_RA_DIR_PIN,
        .cfg1Pin = MOTOR_RA_CFG1_PIN,
        .cfg2Pin = MOTOR_RA_CFG2_PIN,
        .maxA = MOTOR_MAX_A,
        .maxV = MOTOR_MAX_V,
        .brakeA = MOTOR_BRAKE_A,
        .aPosK = MOTOR_A_POS_K,
        .gotoMinV = MOTOR_GOTO_MIN_V,
        .minStepI = MOTOR_MIN_STEP_I_MICROS
    };
    m2 = motor_create(m2Cfg);

    ESP_LOGI(TAG, "Motor task started");
    int64_t tLastUpdate = esp_timer_get_time();
    

#ifdef MEASURE_CYCLE_T
    int64_t maxExecT = 0;
#endif
    for(;;) {
#ifdef MEASURE_CYCLE_T
        int64_t t1 = esp_timer_get_time();
#endif
        motor_run(m1);
        motor_run(m2);
        int64_t t2 = esp_timer_get_time();
#ifdef MEASURE_CYCLE_T
        if (maxExecT < t2 - t1)
            maxExecT = t2 - t1;
#endif
        if (t2 > tLastUpdate + MOTOR_TSK_UPADTE_P) {
            tLastUpdate = t2;
            uint64_t time;
            mount_getTime(&time);
            mount_setPos(m1->pos, m2->pos);
            processQueue(time);

            if (tracking) {
                updateTracking(time);
            }
            updateState();
#ifdef MEASURE_CYCLE_T
            uint64_t posOffset = motor_getPosOffset(m1, t2);
            ESP_LOGD(TAG, "M max exec t: %lli micros, posOffset: %lli, mode: %i, v: %f, p: %lli, tpos: %lli", maxExecT, posOffset, m1->mode, m1->v, m1->pos, m1->tPos);
            maxExecT = 0;
#endif
        }
    }

    motor_destroy(m1);
}