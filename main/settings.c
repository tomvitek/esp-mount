#include "settings.h"
#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_log.h>

#define TAG "settings"
#define MAX_TIME_SEMAPHORE_DELAY 1000 // ticks
#define TRACK_BUFFER_SIZE 6000

SemaphoreHandle_t timeMutex;
SemaphoreHandle_t posMutex;
SemaphoreHandle_t tbMutex;

struct MountSettings {
    uint64_t timeOffset;
    int32_t posAx1;
    int32_t posAx2;
} settings;

TrackPoint trackBuffer[TRACK_BUFFER_SIZE];
uint32_t tbTailIdx = 0;
uint32_t tbHeadIdx = 0;

void mount_initSettings() {
    timeMutex = xSemaphoreCreateMutex();
    posMutex = xSemaphoreCreateMutex();
    tbMutex = xSemaphoreCreateMutex();
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

uint8_t mount_pushTrackPoint(TrackPoint tp) {
    if (xSemaphoreTake(tbMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        if (tbTailIdx == tbHeadIdx + 1 || (tbTailIdx == 0 && tbHeadIdx == TRACK_BUFFER_SIZE - 1)) {
            return MOUNT_BUFFER_FULL;
        }

        trackBuffer[tbHeadIdx] = tp;
        tbHeadIdx++;
        if (tbHeadIdx == TRACK_BUFFER_SIZE)
            tbHeadIdx = 0;
        xSemaphoreGive(tbMutex);
        return MOUNT_BUFFER_OK;
    }
    return MOUNT_MTX_ACQ_FAIL;
}

bool mount_pullTrackPoint(TrackPoint *trackPoint) {
    if (xSemaphoreTake(tbMutex, MAX_TIME_SEMAPHORE_DELAY) == pdTRUE) {
        if (tbTailIdx != tbHeadIdx) {
            *trackPoint = trackBuffer[tbTailIdx];
            
            if (++tbTailIdx == TRACK_BUFFER_SIZE)
                tbTailIdx = 0;
            xSemaphoreGive(tbMutex);
            return true;
        }
        
        xSemaphoreGive(tbMutex);
        return false;
    }
    else
        return false;
}

uint32_t mount_getTrackBufferSize() {
    return TRACK_BUFFER_SIZE;
}

uint32_t mount_getTrackBufferFreeSpace() {
    xSemaphoreTake(tbMutex, MAX_TIME_SEMAPHORE_DELAY);
    uint32_t result;
    if (tbTailIdx <= tbHeadIdx)
        result = TRACK_BUFFER_SIZE - tbHeadIdx + tbTailIdx;
    else 
        result = tbTailIdx - tbHeadIdx;
    
    xSemaphoreGive(tbMutex);
    return result;
}

void mount_clearTrackBuffer() {
    xSemaphoreTake(tbMutex, MAX_TIME_SEMAPHORE_DELAY);
    tbTailIdx = 0;
    tbHeadIdx = 0;
    xSemaphoreGive(tbMutex);
}