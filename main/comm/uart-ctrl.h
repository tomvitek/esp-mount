#ifndef __MOUNT_COMM
#define __MOUNT_COMM

#include <driver/uart.h>
#include <driver/gpio.h>
#include "../config.h"
#include "../settings.h"

#define COMM_UART_PORT UART_NUM_2
#define COMM_BAUD_RATE 115200
#define COMM_PIN_TX GPIO_NUM_12
#define COMM_PIN_RX GPIO_NUM_13
#define RX_TX_BUFFER_SIZE 1024

#define MOUNT_MSG_CMD_ERR_UNKNOWN_CMD -3
#define MOUNT_MSG_CMD_ERR_INVALID_CMD -2
#define MOUNT_MSG_CMD_ERR_INVALID_PARAM -1
#define MOUNT_MSG_CMD_NONE 0
#define MOUNT_MSG_CMD_TIME_SYNC 1
#define MOUNT_MSG_CMD_SET_POS 2
#define MOUNT_MSG_CMD_GET_POS 3
#define MOUNT_MSG_CMD_GET_TIME 4
#define MOUNT_MSG_CMD_STOP 5
#define MOUNT_MSG_CMD_GOTO 6
#define MOUNT_MSG_CMD_GET_CPR 7
#define MOUNT_MSG_CMD_GET_STATUS 8
#define MOUNT_MSG_CMD_GET_PROTOCOL_VERSION 9
#define MOUNT_MSG_CMD_GET_TRACK_BUF_FREE_SPACE 10
#define MOUNT_MSG_CMD_GET_TRACK_BUF_SIZE 11
#define MOUNT_MSG_CMD_TRACK_BUF_CLEAR 12
#define MOUNT_MSG_CMD_TRACK_ADD_POINT 13
#define MOUNT_MSG_CMD_TRACKING_BEGIN 14
#define MOUNT_MSG_CMD_TRACKING_STOP 15

#define MOUNT_ERR_CODE_INTERNAL 1
#define MOUNT_ERR_CODE_INVALID_MSG 2
#define MOUNT_ERR_CODE_UNKNOWN_CMD 3

#define MOUNT_STATUS_CODE_STOPPED 0
#define MOUNT_STATUS_CODE_GOTO 1
#define MOUNT_STATUS_CODE_TRACKING 2
#define MOUNT_STATUS_CODE_BRAKING 3

#define UART_CTRL_PROTOCOL_VERSION 0

typedef int cmd_t;
typedef int mount_status_t;
typedef struct MountMsg_SetPos {
    step_t ax1;
    step_t ax2;
} MountMsg_SetPos;

typedef struct MountMsg_Goto {
    step_t ax1;
    step_t ax2;
} MountMsg_Goto;

typedef union MountMsg_data {
    uint64_t time;
    MountMsg_SetPos setPos;
    MountMsg_Goto goTo;
    /**
     * @brief True if the stop should be instant
     * 
     * Associated with `MOUNT_MSG_CMD_STOP` command
     */
    bool stopInstant;
    TrackPoint trackPoint;

} MountMsg_data;

typedef struct MountMsg {
    cmd_t cmd;
    MountMsg_data data;
} MountMsg;

/**
 * @brief Initiates mount communication through a UART port. Other functions in this file can be then used 
 * for communication with the control pc (or some other device). For this communication, pins `COMM_PIN_TX` and `COMM_PIN_RX` are used
 * 
 */
void comm_init();
/**
 * @brief If there is any incoming data to read, reads it and parses it into `MountMsg` object, which is than returned.
 * 
 * If there is no incoming data, MountMsg object with attribute `cmd` set to `MOUNT_MSG_CMD_NONE` is returned.
 * @return MountMsg Mount message object containg command and its arguments.
 */
MountMsg comm_getNext();

/**
 * @brief Sends error back through uart. This error has format of `"! {errCode} {msg}"`
 * 
 * @param errCode Error code. Should be one of MOUNT_ERR_CODE_* constants
 * @param msg Error message. Can be custom, but shouldn't be longer than circa 150 chars.
 */
void comm_sendError(int errCode, const char* msg);

/**
 * @brief Sends time response. This response corresponds to `MOUNT_MSG_CMD_TIME_SYNC` and `MOUNT_MSG_CMD_GET_TIME` commands
 * 
 * @param currentTime Current mount time, in milliseconds.
 */
void comm_sendTimeResponse(uint64_t currentTime);
void comm_sendSetPosResponse(step_t ax1, step_t ax2);
void comm_sendGetPosResponse(step_t ax1, step_t ax2);
void comm_sendGotoResponse(step_t ax1, step_t ax2);

/**
 * @brief Sends a response to the stop command
 * 
 * @param instant True if the stop was instant
 */
void comm_sendStopResponse(bool instant);
/**
 * @brief Sends a response to the cpr request command
 * 
 * @param ax1 The first axis CPR
 * @param ax2 The second axis CPR
 */
void comm_sendCprResponse(step_t ax1, step_t ax2);

/**
 * @brief Sends a response to the status request command
 * 
 * @param status Status of the mount (one of MOUNT_STATUS_CODE_* constants)
 */
void comm_sendStatusResponse(mount_status_t status);

/**
 * @brief Sends a response to the protocol version request command
 * 
 */
void comm_sendProtocolVersionResponse();

void comm_sendTrackBufferFreeSpaceResponse(uint32_t freeSpace);
void comm_sendTrackBufferSizeResponse(uint32_t size);
void comm_sendTrackBufferClearResponse();
void comm_sendAddTrackPointResponse(uint8_t successCode);
void comm_sendTrackingBeginResponse();
void comm_sendTrackingStopRespone();
#endif