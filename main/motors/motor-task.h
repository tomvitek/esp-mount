#ifndef __MOTOR_TASK
#define __MOTOR_TASK

#include <driver/gpio.h>

#define MOTOR_STEP_PIN GPIO_NUM_25
#define MOTOR_DIR_PIN GPIO_NUM_26
#define MOTOR_CFG1_PIN GPIO_NUM_18
#define MOTOR_CFG2_PIN GPIO_NUM_27
#define MOTOR_MIN_STEP_I_MICROS 700
#define MOTOR_MAX_V 13000.0f
#define MOTOR_MAX_A 2500.0f
#define MOTOR_A_POS_K 50.0f
#define MOTOR_BRAKE_A 2500
#define MOTOR_GOTO_MIN_V 100.0f
#define MOTOR_TSK_UPADTE_P 200000
void motor_task(void* args);

#endif