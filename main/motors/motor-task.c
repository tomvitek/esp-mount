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
        .gotoMinV = 100.0f
    };
    motor_t motor1 = motor_create(m1Cfg);
    ESP_LOGI(TAG, "Motor task started");
    int64_t timeGoal = esp_timer_get_time() + 15000000;
    motor_track(motor1, 0, 10000, esp_timer_get_time(), timeGoal);
    //motor_goto(motor1, -20000);
    int64_t tLastUpdate = esp_timer_get_time();
    for(;;) {
        int64_t t1 = esp_timer_get_time();
        motor_run(motor1);
        int64_t t2 = esp_timer_get_time();
        if (t2 > tLastUpdate + MOTOR_TSK_UPADTE_P) {
            tLastUpdate = t2;
            uint64_t posOffset = motor_getPosOffset(motor1, t1);
            int64_t time = esp_timer_get_time();
            //motor_track(motor1, 0, 1000, time, time + MOTOR_TSK_UPADTE_P);
            //motor_goto(motor1, 1000);
            ESP_LOGD(TAG, "M run exec t: %lli micros, posOffset: %lli, mode: %i, v: %f, p: %lli", t2 - t1, posOffset, motor1->mode, motor1->v, motor1->pos);
        }
    }

    motor_destroy(motor1);
}