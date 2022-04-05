#ifndef __MOTOR_DRIVER
#define __MOTOR_DRIVER
#include "../settings.h"
#include "../config.h"

typedef enum motor_mode {
    STOP,
    TRACKING,
    GOTO
} motor_mode_t;

typedef struct MotorConfig {
    int stepPin;
    int dirPin;
    int cfg1Pin;
    int cfg2Pin;

    float maxA;
    float maxV;
    float brakeA;
    float aPosK;
    float gotoMinV;
    /**
     * @brief Minimum step interval allowed. When this step interval is reached, motor driver will try to 
     * switch to a lower step resolution (and therefore higher speed per step interval)
     */
    int32_t minStepI;
} motor_config_t;

typedef struct Motor {
    
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
    int64_t lastParamUpdateTime;
    /**
     * @brief Time on the latest track command (in microseconds)
     * 
     */
    int64_t tStartTime;
    /**
     * @brief Position on the latest track command (in microseconds)
     * 
     */
    step_t tStartPos;
    int64_t tStartV;
    /**
     * @brief Current step interval (im microseconds)
     * 
     */
    float v;
    float stepIOffsetCounter;
    int64_t lastStepIChangeTime;
    uint8_t dir;
    motor_mode_t mode;
    uint8_t multIdx;
    motor_config_t cfg;
} Motor;

typedef Motor* motor_t;

motor_t motor_create(motor_config_t config);
/**
 * @brief Initiates motor tracking. During tracking, the motor will try to interpolate between startPos and endPos, with speed linearly 
 * increasing/decreasing based on the speed on the start of tracking.
 * 
 * Ideally, the motor should be already on the startPos when the tracking starts.
 * 
 * @param motor Motor
 * @param startPos Tracking starting position
 * @param targetPos Tracking target postition
 * @param startTime Time when the tracking starts
 * @param targetTime Time when the motor should reach the target position
 */
void motor_track(motor_t motor, step_t startPos, step_t targetPos, int64_t startTime, int64_t targetTime);
void motor_goto(motor_t motor, step_t targetPos);
void motor_run(motor_t motor);
void motor_destroy(motor_t motor);
int64_t motor_getPosOffset(motor_t m, int64_t t);
#endif