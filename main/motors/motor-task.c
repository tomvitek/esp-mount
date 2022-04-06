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

void motor_task(void *args) {
    QueueHandle_t motorCmdQueue = args;

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
    motor_t m1 = motor_create(m1Cfg);

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
    motor_t m2 = motor_create(m2Cfg);

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
                }
                else if (cmd.type == CMD_STOP) {
                    motor_stop(m1, cmd.data.instantStop);
                    motor_stop(m2, cmd.data.instantStop);
                }
            }

#ifdef MEASURE_CYCLE_T
            uint64_t posOffset = motor_getPosOffset(m1, t2);
            ESP_LOGD(TAG, "M max exec t: %lli micros, posOffset: %lli, mode: %i, v: %f, p: %lli, tpos: %lli", maxExecT, posOffset, m1->mode, m1->v, m1->pos, m1->tPos);
            maxExecT = 0;
#endif
        }
    }

    motor_destroy(m1);
}