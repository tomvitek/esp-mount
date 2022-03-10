#ifndef __MOUNT_TIME
#define __MOUNT_TIME

#include <stdio.h>
#include <stdbool.h>

void mount_initSettings();

bool mount_getTime(uint64_t *time);
bool mount_setTime(uint64_t time);

bool mount_getPos(int32_t* ax1, int32_t *ax2);
bool mount_setPos(int32_t ax1, int32_t ax2);
#endif