#include "motor-task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "../settings.h"
#include <esp_timer.h>
#include "motor-driver.h"

#define TAG "motor-task"

void motor_task(void *args) {

    motor_t motor1 = motor_create(MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_CFG1_PIN, MOTOR_CFG2_PIN, MOTOR_MAX_V, MOTOR_MAX_V, MOTOR_A_POS_K);
    ESP_LOGI(TAG, "Motor task started");
    int64_t timeGoal = esp_timer_get_time() + 80000000;
    motor_goto(motor1, -100000, timeGoal);
    int64_t tLastUpdate = esp_timer_get_time();
    for(;;) {
        int64_t t1 = esp_timer_get_time();
        motor_run(motor1);
        int64_t t2 = esp_timer_get_time();
        if (t2 > tLastUpdate + MOTOR_TSK_UPADTE_P) {
            tLastUpdate = t2;
            uint64_t posOffset = motor_getPosOffset(motor1, t1);
            ESP_LOGD(TAG, "M run exec t: %lli micros, posOffset: %lli", t2 - t1, posOffset);
        }
    }

    motor_destroy(motor1);
}