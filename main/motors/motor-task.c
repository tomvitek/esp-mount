#include "motor-task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "../settings.h"
#include <esp_timer.h>
#include "motor-driver.h"
#include <math.h>
#define TAG "motor-task"

void motor_task(void *args) {
    motor_config_t m1Cfg = {
        .stepPin = MOTOR_STEP_PIN,
        .dirPin = MOTOR_DIR_PIN,
        .cfg1Pin = MOTOR_CFG1_PIN,
        .cfg2Pin = MOTOR_CFG2_PIN,
        .maxA = MOTOR_MAX_A,
        .maxV = MOTOR_MAX_V,
        .brakeA = MOTOR_BRAKE_A,
        .aPosK = MOTOR_A_POS_K,
        .gotoMinV = MOTOR_GOTO_MIN_V,
        .minStepI = MOTOR_MIN_STEP_I_MICROS
    };
    motor_t motor1 = motor_create(m1Cfg);
    ESP_LOGI(TAG, "Motor task started");
    int64_t timeGoal = esp_timer_get_time() + 15000000;
    motor_track(motor1, 500, 10000, esp_timer_get_time(), timeGoal);
    //motor_goto(motor1, -200000);
    int64_t tLastUpdate = esp_timer_get_time();
    int64_t maxExecT = 0;
    for(;;) {
        int64_t t1 = esp_timer_get_time();
        motor_run(motor1);
        int64_t t2 = esp_timer_get_time();
        if (maxExecT < t2 - t1)
            maxExecT = t2 - t1;
        if (t2 > tLastUpdate + MOTOR_TSK_UPADTE_P) {
            tLastUpdate = t2;
            uint64_t posOffset = motor_getPosOffset(motor1, t1);
            time_t time;
            int64_t motorT = esp_timer_get_time();
            //ESP_LOGD(TAG, "M max exec t: %lli micros, posOffset: %lli, mode: %i, v: %f, p: %lli, tpos: %lli", maxExecT, posOffset, motor1->mode, motor1->v, motor1->pos, motor1->tPos);
            maxExecT = 0;
            mount_getTime(&time);
            motor_track(motor1, time * 5 - MOTOR_TSK_UPADTE_P / 1000 * 5, time * 5, motorT, motorT + MOTOR_TSK_UPADTE_P);
        }
    }

    motor_destroy(motor1);
}