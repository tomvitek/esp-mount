#include "comm-task.h"
#include "uart-ctrl.h"
#include <esp_log.h>
#include "../settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../config.h"
#define TAG "comm-task"

void comm_task() {
    comm_init();

    TickType_t lastTicks = xTaskGetTickCount();
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
            ESP_LOGI(TAG, "Received goto msg: [%lli %lli] (time %llu)", gotoData.ax1, gotoData.ax2, gotoData.timeIncluded ? gotoData.time : 0);
            comm_sendGotoResponse(gotoData.ax1, gotoData.ax2, gotoData.timeIncluded ? &gotoData.time : NULL);
        }
        else if (msg.cmd == MOUNT_MSG_CMD_STOP) {
            ESP_LOGI(TAG, "Received stop msg (instant: %hhi)", msg.data.stopInstant);
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