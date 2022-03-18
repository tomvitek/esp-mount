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
#define LED_PIN GPIO_NUM_2
#define LED2_PIN GPIO_NUM_33
#define CORE_MOTORS 1
#define TAG "main"

QueueHandle_t blinkQueue = 0;

void blink_task(void *args) {
    blinkQueue = xQueueCreate(10, sizeof(uint32_t));

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    TaskHandle_t idleTaskHandle = xTaskGetIdleTaskHandleForCPU(1);
    esp_task_wdt_delete(idleTaskHandle);
    //uint64_t counter = 0;
    //TickType_t lastTicks = xTaskGetTickCount();
    ESP_LOGD("blink", "Starting blink task");
    for(;;) {
        //vTaskDelayUntil(&startTime, DELAY_MS / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 1);
        //vTaskDelayUntil(&startTime, DELAY_MS / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        //ESP_LOGI("app_main", "blink");

        uint32_t queueMsg;
        if (xQueueReceive(blinkQueue, &queueMsg, 0) == pdTRUE) {
            //ESP_LOGI("blink", "Received msg %i", queueMsg);
        }
    }
}

void blink2_task(void *args) {
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    
    TaskHandle_t idleTaskHandle = xTaskGetIdleTaskHandleForCPU(1);
    esp_task_wdt_delete(idleTaskHandle);
    uint32_t counter = 0;
    TickType_t lastTicks = xTaskGetTickCount();
    //ESP_LOGI("blink", "Starting blink task");
    for(;;) {
        vTaskDelayUntil(&lastTicks, DELAY_MS / portTICK_PERIOD_MS);
        gpio_set_level(LED2_PIN, 1);
        vTaskDelayUntil(&lastTicks, 1);
        gpio_set_level(LED2_PIN, 0);
        uint64_t time;
        mount_getTime(&time);
        //ESP_LOGD("blink2", "current time: %llu", time);
        configSTACK_DEPTH_TYPE depth = uxTaskGetStackHighWaterMark(NULL);
        //ESP_LOGD("blink2", "Remaining stack depth: %u", depth);
        if (blinkQueue != 0) {
            xQueueSend(blinkQueue, &counter, 0);
        }
        counter++;
    }
}

void app_main() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGD("app_main", "hi");
    ESP_LOGD("app_main", "portTICK_PERIOD_MS: %i", portTICK_PERIOD_MS);
    mount_initSettings();
    xTaskCreatePinnedToCore(blink2_task, "blink2", 2500, NULL, tskIDLE_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(comm_task, "commTask", 3000, NULL, tskIDLE_PRIORITY, NULL, 0);
    vTaskDelay(10);
    xTaskCreatePinnedToCore(motor_task, "motorTask", 5000, NULL, 12, NULL, CORE_MOTORS);
}
