#ifndef __MOUNT_TIME
#define __MOUNT_TIME

#include <stdio.h>
#include <stdbool.h>
#include "config.h"

#define MOUNT_BUFFER_OK 0
#define MOUNT_BUFFER_FULL 1
#define MOUNT_MTX_ACQ_FAIL 2
typedef struct TrackPoint {
    step_t ax1;
    step_t ax2;
    uint64_t time;
} TrackPoint;

void mount_initSettings();

bool mount_getTime(uint64_t *time);
bool mount_setTime(uint64_t time);

bool mount_getPos(int32_t* ax1, int32_t *ax2);
bool mount_setPos(int32_t ax1, int32_t ax2);

uint8_t mount_pushTrackPoint(TrackPoint trackPoint);
bool mount_pullTrackPoint(TrackPoint *trackPoint);
uint32_t mount_getTrackBufferSize();
uint32_t mount_getTrackBufferFreeSpace();
void mount_clearTrackBuffer();
#endif