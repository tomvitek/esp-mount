#include "settings.h"
#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_log.h>

#define TAG "settings"
#define MAX_TIME_SEMAPHORE_DELAY 1000 // ticks
SemaphoreHandle_t timeMutex;
SemaphoreHandle_t posMutex;

struct MountSettings {
    uint64_t timeOffset;
    int32_t posAx1;
    int32_t posAx2;
} settings;

void mount_initSettings() {
    timeMutex = xSemaphoreCreateMutex();
    posMutex = xSemaphoreCreateMutex();
    settings.timeOffset = 0;
    settings.posAx1 = 0;
    settings.posAx2 = 0;
}

bool mount_getTime(uint64_t *time) {
    uint64_t localTimeOffset;

    if (xSemaphoreTake(timeMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        localTimeOffset = settings.timeOffset;
        xSemaphoreGive(timeMutex);
    }
    else {
        ESP_LOGW(TAG, "Couldn't acquire time mutex!");
        return false;
    }

    *time = esp_timer_get_time() / 1000 + localTimeOffset;
    return true;
}

bool mount_setTime(uint64_t realTime) {
    int64_t espTime = esp_timer_get_time() / 1000;
    uint64_t localTimeOffset = realTime - espTime;
    if (xSemaphoreTake(timeMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        settings.timeOffset = localTimeOffset;
        xSemaphoreGive(timeMutex);
    }
    else {
        ESP_LOGW(TAG, "Couldn't acquire time mutex!");
        return false;
    }

    return true;
}

bool mount_setPos(int32_t ax1, int32_t ax2) {
    if (xSemaphoreTake(posMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        settings.posAx1 = ax1;
        settings.posAx2 = ax2;
        xSemaphoreGive(posMutex);
        return true;
    }
    else {
        ESP_LOGW(TAG, "Couldn't acquire pos mutex!");
        return false;
    }

    return true;
}

bool mount_getPos(int32_t *ax1, int32_t *ax2) {
    if (xSemaphoreTake(posMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        *ax1 = settings.posAx1;
        *ax2 = settings.posAx2;
        xSemaphoreGive(posMutex);
        return true;
    }
    else {
        ESP_LOGW(TAG, "Couldn't acquire pos mutex!");
        return false;
    }

    return true;
}