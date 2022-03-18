#ifndef __MOTOR_DRIVER
#define __MOTOR_DRIVER
#include "../settings.h"
#include "../config.h"

typedef enum motor_mode {
    STOP,
    TRACKING,
    GOTO
} motor_mode_t;

typedef struct Motor {
    int stepPin;
    int dirPin;
    int cfg1Pin;
    int cfg2Pin;
    step_t pos;
    /**
     * @brief Target position
     * 
     */
    step_t tPos;
    /**
     * @brief Target position reach time
     * 
     */
    int64_t tTime;
    /**
     * @brief Time of the latest step (in microseconds)
     * 
     */
    int64_t lastStepTime;
    /**
     * @brief Time on the latest goto command (in microseconds)
     * 
     */
    int64_t prevTime;
    /**
     * @brief Position on the latest goto command (in microseconds)
     * 
     */
    step_t prevPos;
    int64_t prevStepI;
    /**
     * @brief Current step interval (im microseconds)
     * 
     */
    float stepI;
    float stepIOffsetCounter;
    int64_t lastStepIChangeTime;
    uint8_t dir;
    int64_t maxA;
    int64_t maxV;
    float aPosK;
    motor_mode_t mode;
} Motor;

typedef Motor* motor_t;

motor_t motor_create(int stepPin, int dirPin, int cfg1Pin, int cfg2Pin, int64_t maxA, int64_t maxV, float aPosK);
void motor_goto(motor_t motor, step_t targetPos, int64_t targetTime);
void motor_run(motor_t motor);
void motor_destroy(motor_t motor);
int64_t motor_getPosOffset(motor_t m, int64_t t);
#endif