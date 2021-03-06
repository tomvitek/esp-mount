#include "uart-ctrl.h"
#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/task.h>
#include <string.h>

#define TAG "mount-comm"

#define CMD_START '+'
#define CMD_STR_TIME_SYNC "t"
#define CMD_STR_SET_POS "p"
#define CMD_STR_GET_POS "gp"
#define CMD_STR_GET_TIME "gt"
#define CMD_STR_GOTO "g"
#define CMD_STR_STOP "s"
#define CMD_STR_GET_CPR "gc"
#define CMD_STR_GET_STATUS "gs"
#define CMD_STR_GET_PROTOCOL_VERSION "gpv"
#define CMD_STR_GET_TRACK_BUF_FREE_SPACE "gtbf"
#define CMD_STR_GET_TRACK_BUF_SIZE "gtbs"
#define CMD_STR_TRACK_BUF_CLEAR "tbc"
#define CMD_STR_TRACK_BUF_ADD_POINT "tp"
#define CMD_STR_TRACKING_BEGIN "tb"
#define CMD_STR_TRACKING_STOP "ts"

#define UART_TIMEOUT_MS 10

/**
 * @brief Initiates mount communication through a UART port. Other functions in this file can be then used 
 * for communication with the control pc (or some other device). For this communication, pins `COMM_PIN_TX` and `COMM_PIN_RX` are used
 * 
 */
void comm_init() {
    uart_config_t config = {
        .baud_rate = COMM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(COMM_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_PORT, COMM_PIN_TX, COMM_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(COMM_UART_PORT, RX_TX_BUFFER_SIZE, RX_TX_BUFFER_SIZE, 0, NULL, 0));
    ESP_LOGD(TAG, "Control UART initialized");
}

MountMsg makeMountMsg(cmd_t cmd) {
    MountMsg msg = {
        .cmd = cmd
    };
    return msg;
}

char receive_char(){
    char nextByte;
    uart_read_bytes(COMM_UART_PORT, &nextByte, sizeof(nextByte), 0);
    return nextByte;
}

bool waitForAvailable(size_t availableMin, uint32_t timeout) {
    size_t available = 0;
    uart_get_buffered_data_len(COMM_UART_PORT, &available);
    TickType_t startTick = xTaskGetTickCount();

    while (available < availableMin) {
        vTaskDelay(1);
        uart_get_buffered_data_len(COMM_UART_PORT, &available);
        if (xTaskGetTickCount() - startTick > timeout / portTICK_PERIOD_MS) {
            ESP_LOGW(TAG, "UART command timed out");
            return false;
        }
    }

    return true;
}

/**
 * @brief Reads the next command parameter(something surrounded by space characters, or space and end-of-line characters in case of the last parameter).
 * For example, if the rx buffer contains: " hi how are you\n", `receive_space_block` will return "hi" and the rx buffer will than contain only "how are you\n".
 * 
 * @param buffer Return buffer. The resulting parameter will be written into this buffer
 * @param len Buffer length.
 * @param endFlag Is set to true when the space block is immidately followed by \n character. Can be set to NULL, in which case it is not used.
 *  If it points to `true` in the beginning, `receive_space_block` returns immidiately (because the command has already ended).
 * @return size_t Length of the read string (excluding the 0 byte at the end)
 */
size_t receive_space_block(char* buffer, size_t len, bool *endFlag) {
    if (endFlag != NULL && *endFlag == true){
        return 0; // If the command already ended, returns immidiately
    }
        

    size_t bufferCounter = 0;
    if (waitForAvailable(1, UART_TIMEOUT_MS)) {
        char firstChar = receive_char();
        if (firstChar != ' ') {
            buffer[0] = firstChar;
            bufferCounter++;
        }
        if (firstChar == '\n') {
            ESP_LOGD(TAG, "First char is LF");
            *endFlag = true;
            return 0;
        }
    }

    while (bufferCounter < len - 1 && waitForAvailable(1, UART_TIMEOUT_MS)) {
        char nextChar = receive_char();
        if (nextChar == ' ' || nextChar == '\n'){
            if (nextChar == '\n' && endFlag != NULL){
                *endFlag = true;
            }
            break;
        }
        
        buffer[bufferCounter] = nextChar;
        bufferCounter++;
    }
    buffer[bufferCounter] = 0;

    return bufferCounter;
}

bool receive_uint64(uint64_t *result, bool *endFlag) {
    char intStr[21];
    
    size_t intStrLen = receive_space_block(intStr, sizeof(intStr), endFlag);

    if (intStrLen == 0)
        return false;

    *result = strtoull(intStr, NULL, 10);
    return true;
}

bool receive_int64(int64_t* result, bool *endFlag) {
    char intStr[20];

    size_t intStrLen = receive_space_block(intStr, sizeof(intStr), endFlag);

    if (intStrLen == 0)
        return false;

    *result = strtoll(intStr, NULL, 10);
    return true;
}

/**
 * @brief Receives a boolean from the communication uart port
 * 
 * The boolean is (in the protocol) represented as either "0" (false) or "1" (true)
 * 
 * @param result Read boolean will be save here
 * @param endFlag When the last character read will be \n, sets this to true
 * @return true Success
 * @return false Fail
 */
bool receive_bool(bool* result, bool *endFlag) {
    char boolStr[5];
    size_t boolStrLen = receive_space_block(boolStr, sizeof(boolStr), endFlag);
    
    if (boolStrLen == 0)
        return false;

    *result = atoi(boolStr) > 0;
    return true;
}

bool receive_end() {
    size_t available;
    uart_get_buffered_data_len(COMM_UART_PORT, &available);
    if (available == 0)
        return false;

    char endChar = 0;
    while (waitForAvailable(1, 0)){
        endChar = receive_char();
    }
    
    return endChar == '\n';
}

MountMsg parseTimeSyncCmd(bool *endFlag) {
    uint64_t time;
    bool success = receive_uint64(&time, endFlag);

    if (!success)
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_PARAM);

    if (*endFlag || receive_end()) {
        MountMsg_data data;
        data.time = time;
        MountMsg msg = {
            .cmd = MOUNT_MSG_CMD_TIME_SYNC,
            .data = data
        };
        return msg;
    }

    return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_CMD);
}

MountMsg parseSetPosCmd(bool *endFlag) {
    step_t posRa, posDec;
    bool success = receive_int64(&posRa, endFlag);
    success &= receive_int64(&posDec, endFlag);

    success &= endFlag || receive_end();

    if (!success)
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_PARAM);

    MountMsg_SetPos setPosData = {
        .ax1 = posRa,
        .ax2 = posDec
    };

    MountMsg_data data = {
        .setPos = setPosData
    };

    MountMsg msg = {
        .cmd = MOUNT_MSG_CMD_SET_POS,
        .data = data
    };

    return msg;
}

MountMsg parseGotoMsg(bool *endFlag) {
    step_t posRa, posDec;
    bool success = receive_int64(&posRa, endFlag);
    success &= receive_int64(&posDec, endFlag);
    
    //success &= *endFlag || receive_end();

    if (!success)
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_PARAM);

    MountMsg_Goto gotoData = {
        .ax1 = posRa,
        .ax2 = posDec
    };

    MountMsg_data data = {
        .goTo = gotoData
    };

    MountMsg msg = {
        .cmd = MOUNT_MSG_CMD_GOTO,
        .data = data
    };

    return msg;
}

MountMsg parseStopMsg(bool *endFlag) {
    bool instantStop;
    bool success = receive_bool(&instantStop, endFlag);
    success &= *endFlag || receive_end();
    
    if (!success)
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_PARAM);

    MountMsg_data data = {
        .stopInstant = instantStop
    };

    MountMsg msg = {
        .cmd = MOUNT_MSG_CMD_STOP,
        .data = data
    };

    return msg;
}

MountMsg parseTrackPointMsg(bool *endFlag) {
    step_t ax1, ax2;
    uint64_t time;
    bool success = receive_int64(&ax1, endFlag);
    success &= receive_int64(&ax2, endFlag);
    success &= receive_uint64(&time, endFlag);

    if (!success)
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_PARAM);

    TrackPoint trackPoint = {
        .ax1 = ax1,
        .ax2 = ax2,
        .time = time
    };

    MountMsg msg = {
        .cmd = MOUNT_MSG_CMD_TRACK_ADD_POINT,
        .data = {
            .trackPoint = trackPoint
        }
    };

    return msg;
}

MountMsg returnWithEFCheck(MountMsg mainMsg, bool *endFlag) {
    if (! *endFlag) {
        receive_end();
        return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_CMD);
    }
    return mainMsg;
}

MountMsg comm_getNext() {
    size_t available = 0;
    uart_get_buffered_data_len(COMM_UART_PORT, &available);
    if (available > 2) {
        char currentByte;

        uart_read_bytes(COMM_UART_PORT, &currentByte, sizeof(currentByte), 0);
        if (currentByte != CMD_START) {
            uart_flush_input(COMM_UART_PORT); // clear input buffer, so no data from the invalid command persist
            
            return makeMountMsg(MOUNT_MSG_CMD_ERR_INVALID_CMD);
        }

        char cmdBuffer[10];
        bool endFlag = false;
        receive_space_block(cmdBuffer, sizeof(cmdBuffer), &endFlag);

        if (strcmp(cmdBuffer, CMD_STR_TIME_SYNC) == 0) {
            return parseTimeSyncCmd(&endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_SET_POS) == 0) {
            return parseSetPosCmd(&endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_POS) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_POS), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_TIME) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_TIME), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_CPR) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_CPR), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_STATUS) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_STATUS), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_PROTOCOL_VERSION) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_PROTOCOL_VERSION), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GOTO) == 0) {
            return parseGotoMsg(&endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_STOP) == 0) {
            return parseStopMsg(&endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_TRACK_BUF_FREE_SPACE) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_TRACK_BUF_FREE_SPACE), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_GET_TRACK_BUF_SIZE) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_GET_TRACK_BUF_SIZE), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_TRACK_BUF_CLEAR) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_TRACK_BUF_CLEAR), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_TRACK_BUF_ADD_POINT) == 0) {
            return parseTrackPointMsg(&endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_TRACKING_BEGIN) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_TRACKING_BEGIN), &endFlag);
        }
        else if (strcmp(cmdBuffer, CMD_STR_TRACKING_STOP) == 0) {
            return returnWithEFCheck(makeMountMsg(MOUNT_MSG_CMD_TRACKING_STOP), &endFlag);
        }
        else {
            if (!endFlag)
                receive_end();
            ESP_LOGW(TAG, "Unknown command received");
            return makeMountMsg(MOUNT_MSG_CMD_ERR_UNKNOWN_CMD);
        }
    }
    
    return makeMountMsg(MOUNT_MSG_CMD_NONE);
}

void comm_sendTimeResponse(uint64_t currentTime) {
    char msg[30];
    snprintf(msg, sizeof(msg), "+%s %llu\n", CMD_STR_TIME_SYNC, currentTime);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendGetTimeResponse(uint64_t currentTime) {
    char msg[30];
    snprintf(msg, sizeof(msg), "+%s %llu\n", CMD_STR_GET_TIME, currentTime);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendSetPosResponse(step_t posAx1, step_t posAx2) {
    char msg[30];
    snprintf(msg, sizeof(msg), "+p %lli %lli\n", posAx1, posAx2);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendGetPosResponse(step_t ax1, step_t ax2) {
    char msg[30];
    snprintf(msg, sizeof(msg), "+%s %lli %lli\n", CMD_STR_GET_POS, ax1, ax2);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendError(int errCode, const char* msg) {
    char msgBuffer[200];
    snprintf(msgBuffer, sizeof(msgBuffer), "! %i %s\n", errCode, msg);
    uart_write_bytes(COMM_UART_PORT, msgBuffer, strlen(msgBuffer));
}

void comm_sendGotoResponse(step_t ax1, step_t ax2) {
    char msg[60];
    snprintf(msg, sizeof(msg), "+g %lli %lli\n", ax1, ax2);

    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendStopResponse(bool instant) {
    char msg[20];
    snprintf(msg, sizeof(msg), "+s %hhi\n", instant);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendCprResponse(step_t ax1, step_t ax2) {
    char msg[40];
    snprintf(msg, sizeof(msg), "+gc %lli %lli\n", ax1, ax2);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendStatusResponse(mount_status_t status) {
    char msg[20];
    snprintf(msg, sizeof(msg), "+gs %i\n", status);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendProtocolVersionResponse() {
    char msg[20];
    snprintf(msg, sizeof(msg), "+gpv %i\n", UART_CTRL_PROTOCOL_VERSION);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendTrackBufferFreeSpaceResponse(uint32_t freeSpace) {
    char msg[20];
    snprintf(msg, sizeof(msg), "+%s %u\n", CMD_STR_GET_TRACK_BUF_FREE_SPACE, freeSpace);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendTrackBufferSizeResponse(uint32_t size) {
    char msg[20];
    snprintf(msg, sizeof(msg), "+%s %u\n", CMD_STR_GET_TRACK_BUF_SIZE, size);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void sendEmptyResponse(char* cmdStr) {
    char msg[10];
    snprintf(msg, sizeof(msg), "+%s\n", cmdStr);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendTrackBufferClearResponse() {
    sendEmptyResponse(CMD_STR_TRACK_BUF_CLEAR);
}

void comm_sendAddTrackPointResponse(uint8_t successCode) {
    char msg[20];
    snprintf(msg, sizeof(msg), "+%s %hu\n", CMD_STR_TRACK_BUF_ADD_POINT, successCode);
    uart_write_bytes(COMM_UART_PORT, msg, strlen(msg));
}

void comm_sendTrackingBeginResponse() {
    sendEmptyResponse(CMD_STR_TRACKING_BEGIN);
}

void comm_sendTrackingStopRespone() {
    sendEmptyResponse(CMD_STR_TRACKING_STOP);
}