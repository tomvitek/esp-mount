#ifndef __MOTOR_TASK
#define __MOTOR_TASK

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "../config.h"
#define MOTOR_DEC_STEP_PIN GPIO_NUM_25
#define MOTOR_DEC_DIR_PIN GPIO_NUM_26
#define MOTOR_DEC_CFG1_PIN GPIO_NUM_18
#define MOTOR_DEC_CFG2_PIN GPIO_NUM_27
#define MOTOR_MIN_STEP_I_MICROS 350
#define MOTOR_MAX_V 20000.0f
#define MOTOR_MAX_A 2500.0f
#define MOTOR_A_POS_K 50.0f
#define MOTOR_BRAKE_A 2500
#define MOTOR_GOTO_MIN_V 100.0f
#define MOTOR_TSK_UPADTE_P 200000

#define MOTOR_RA_STEP_PIN GPIO_NUM_19
#define MOTOR_RA_DIR_PIN GPIO_NUM_20
#define MOTOR_RA_CFG1_PIN GPIO_NUM_21
#define MOTOR_RA_CFG2_PIN GPIO_NUM_22

typedef enum MotorCmdType {
    CMD_POSITION_UPDATE,
    CMD_GOTO,
    CMD_STOP,
    CMD_TRACK_BEGIN,
    CMD_TRACK_STOP
} MotorCmdType;

typedef struct MotorPosData {
    step_t ax1;
    step_t ax2;
} MotorPosData;

typedef union MotorCmdData {
    MotorPosData pos;
    bool instantStop;
} MotorCmdData;
typedef struct MotorCmd {
    MotorCmdType type;
    MotorCmdData data;
} MotorCmd; 

void motor_task(void* args);

#endif