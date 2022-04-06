#include "comm-task.h"
#include "uart-ctrl.h"
#include <esp_log.h>
#include "../settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../config.h"
#include "../motors/motor-task.h"
#define TAG "comm-task"

void comm_task(void *args) {
    comm_init();

    TickType_t lastTicks = xTaskGetTickCount();
    QueueHandle_t motorCmdQueue = args;
    for(;;) {
        vTaskDelayUntil(&lastTicks, COMM_TASK_PERIOD / portTICK_PERIOD_MS);
        MountMsg msg = comm_getNext();
        
        if (msg.cmd == MOUNT_MSG_CMD_NONE) {}
        else if (msg.cmd == MOUNT_MSG_CMD_TIME_SYNC) {
            mount_setTime(msg.data.time);
            uint64_t mountTime;
            mount_getTime(&mountTime);
            ESP_LOGI(TAG, "Received time %llu", mountTime);
            comm_sendTimeResponse(mountTime);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_SET_POS) {
            MountMsg_SetPos pos = msg.data.setPos;
            mount_setPos(pos.ax1, pos.ax2);
            ESP_LOGI(TAG, "Received new pos: [%lli %lli]", pos.ax1, pos.ax2);
            MotorPosData data = {
                .ax1 = pos.ax1,
                .ax2 = pos.ax2
            };

            MotorCmd cmd = {
                .type = CMD_POSITION_UPDATE,
                .data = {
                    .pos = data
                }
            };
            xQueueSend(motorCmdQueue, &cmd, 0);
            comm_sendSetPosResponse(pos.ax1, pos.ax2);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_POS) {
            int32_t ax1, ax2;
            if (mount_getPos(&ax1, &ax2))
                comm_sendGetPosResponse(ax1, ax2);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_TIME) {
            uint64_t time;
            mount_getTime(&time);
            comm_sendTimeResponse(time);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GOTO) {
            MountMsg_Goto gotoData = msg.data.goTo;
            ESP_LOGI(TAG, "Received goto msg: [%lli %lli]", gotoData.ax1, gotoData.ax2);
            MotorPosData data = {
                .ax1 = msg.data.goTo.ax1,
                .ax2 = msg.data.goTo.ax2
            };

            MotorCmd cmd = {
                .type = CMD_GOTO,
                .data = {
                    .pos = data
                }
            };

            xQueueSend(motorCmdQueue, &cmd, 0);
            comm_sendGotoResponse(gotoData.ax1, gotoData.ax2);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_STOP) {
            ESP_LOGI(TAG, "Received stop msg (instant: %hhi)", msg.data.stopInstant);
            MotorCmd cmd = {
                .type = CMD_STOP,
                .data = {
                    .instantStop = msg.data.stopInstant
                }
            };

            xQueueSend(motorCmdQueue, &cmd, 0);
            comm_sendStopResponse(msg.data.stopInstant);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_CPR) {
            ESP_LOGI(TAG, "Received cpr request");
            comm_sendCprResponse(CPR_AX1, CPR_AX2);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_STATUS) {
            ESP_LOGI(TAG, "Received status request");
            comm_sendStatusResponse(MOUNT_STATUS_CODE_STOPPED);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_PROTOCOL_VERSION) {
            ESP_LOGI(TAG, "Requested protocol version");
            comm_sendProtocolVersionResponse();
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_TRACK_BUF_FREE_SPACE) {
            ESP_LOGI(TAG, "Requested track buffer free space");
            comm_sendTrackBufferFreeSpaceResponse(mount_getTrackBufferFreeSpace());
        }
        else if (msg.cmd == MOUNT_MSG_CMD_GET_TRACK_BUF_SIZE) {
            ESP_LOGI(TAG, "Requested track buffer size");
            comm_sendTrackBufferSizeResponse(mount_getTrackBufferSize());
        }
        else if (msg.cmd == MOUNT_MSG_CMD_TRACK_BUF_CLEAR) {
            ESP_LOGI(TAG, "Requested track buffer clear");
            mount_clearTrackBuffer();
            comm_sendTrackBufferClearResponse();
        }
        else if (msg.cmd == MOUNT_MSG_CMD_TRACK_ADD_POINT) {
            ESP_LOGI(TAG, "Received new track point [%lli %lli %llu]", msg.data.trackPoint.ax1, msg.data.trackPoint.ax2, msg.data.trackPoint.time);
            uint8_t successCode = mount_pushTrackPoint(msg.data.trackPoint);
            comm_sendAddTrackPointResponse(successCode);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_TRACKING_BEGIN) {
            ESP_LOGI(TAG, "Received track begin request");
            // TODO
            comm_sendTrackingBeginResponse();
        }
        else if (msg.cmd == MOUNT_MSG_CMD_TRACKING_STOP) {
            ESP_LOGI(TAG, "Received track stop reqeust");
            // TODO
            comm_sendTrackingStopRespone();
        }
        else if (msg.cmd == MOUNT_MSG_CMD_ERR_INVALID_CMD) {
            comm_sendError(MOUNT_ERR_CODE_INVALID_MSG, "Invalid command received");
        }
        else if (msg.cmd == MOUNT_MSG_CMD_ERR_INVALID_PARAM) {
            comm_sendError(MOUNT_ERR_CODE_INVALID_MSG, "Invalid parameter received");
        }
        else if (msg.cmd == MOUNT_MSG_CMD_ERR_UNKNOWN_CMD) {
            comm_sendError(MOUNT_ERR_CODE_UNKNOWN_CMD, "Unknown command received");
        }
        else {
            comm_sendError(MOUNT_ERR_CODE_INTERNAL, "Unimplemented command");
        }
    }
}