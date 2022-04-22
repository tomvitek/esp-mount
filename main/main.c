/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_int_wdt.h>
#include "settings.h"
#include "comm/comm-task.h"
#include "motors/motor-task.h"

#define DELAY_MS 1000
#define LED_PIN GPIO_NUM_25
#define CORE_MOTORS 1
#define TAG "main"

QueueHandle_t motorCmdQueue = NULL;

void blink_task(void *args) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    uint32_t counter = 0;
    TickType_t lastTicks = xTaskGetTickCount();

    for (int i = 0; i < 3; ++i) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    //ESP_LOGI("blink", "Starting blink task");
    for(;;) {
        vTaskDelayUntil(&lastTicks, DELAY_MS / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        vTaskDelayUntil(&lastTicks, 1);
        gpio_set_level(LED_PIN, 0);
        uint64_t time;
        mount_getTime(&time);

        counter++;
    }
}

void app_main() {
    motorCmdQueue = xQueueCreate(10, sizeof(MotorCmd));
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGD("app_main", "hi");
    ESP_LOGD("app_main", "portTICK_PERIOD_MS: %i", portTICK_PERIOD_MS);
    mount_initSettings();
    xTaskCreatePinnedToCore(blink_task, "blink", 2500, NULL, tskIDLE_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(comm_task, "commTask", 3000, motorCmdQueue, tskIDLE_PRIORITY, NULL, 0);
    vTaskDelay(10);
    xTaskCreatePinnedToCore(motor_task, "motorTask", 5000, motorCmdQueue, 12, NULL, CORE_MOTORS);
}
