#ifndef __MOTOR_DRIVER
#define __MOTOR_DRIVER
#include "../settings.h"
#include "../config.h"

typedef enum motor_mode {
    STOP,
    TRACKING,
    GOTO
} motor_mode_t;

/**
 * @brief Motor configuration. This structure is used for motor initialization by motor_create.
 * 
 */
typedef struct MotorConfig {
    /**
     * @brief STP pin number (for example GPIO_NUM_14)
     * 
     */
    int stepPin;
    /**
     * @brief DIR pin number
     * 
     */
    int dirPin;
    /**
     * @brief CFG1 pin number
     * 
     */
    int cfg1Pin;
    /**
     * @brief CFG2 pin number
     * 
     */
    int cfg2Pin;

    /**
     * @brief Maximum motor acceleration (steps per second squared)
     * 
     */
    float maxA;
    /**
     * @brief Maximum motor velocity (steps per second)
     * 
     */
    float maxV;
    /**
     * @brief Maximum goto acceleration
     * 
     */
    float brakeA;
    /**
     * @brief Acceleration increase for each step off from correct position in tracking mode.
     * 
     */
    float aPosK;
    /**
     * @brief Minimal velocity in GOTO mode (= from this velocity it is safe to stop)
     * 
     */
    float gotoMinV;
    /**
     * @brief Minimum step interval allowed. When this step interval is reached, motor driver will try to 
     * switch to a lower step resolution (and therefore higher speed per step interval)
     */
    int32_t minStepI;
} motor_config_t;

/**
 * @brief Structure containing all the info about a stepper motor - its configuration, position, speed etc.
 * 
 * It is made to control TMC2130 stepper motor driver (tested on BIGTREETECH module)
 */
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
    /**
     * @brief Last time the motor's parameters were recalculated.
     * 
     * Used to ensure the parameters will get updated fast enough even if the motor runs at low speed
     * (at default, the motor recalculates parameters like velocity each step, but when 
     * the step period is e.g. several seconds, it might not be fast enough)
     * 
     */
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
    /**
     * @brief Direction of the motor, either 1 (forward) or 0 (backward)
     * 
     */
    uint8_t dir;
    /**
     * @brief Current motor mode.
     * 
     */
    motor_mode_t mode;
    /**
     * @brief Multiplier index.
     * 
     * Number of the multiplier used for the steps. It works sort of like opposite of step resolution - the more, the smaller the resolution is.
     * Each step's position change (default 1) is multiplied by MULTIPLIERS[multIdx]. (MULTIPLIERS is defined in motor-driver.c)
     * 
     */
    uint8_t multIdx;
    /**
     * @brief Motor configuration
     * 
     */
    motor_config_t cfg;
} Motor;

/**
 * @brief Motor type, pointer to the Motor structure
 * 
 */
typedef Motor* motor_t;

/**
 * @brief Initializes a new motor, sets its pin to correct states and makes it stopped.
 * 
 * @param config 
 * @return motor_t 
 */
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

/**
 * @brief Initiates motor GOTO mode. In this mode, the motor will accelerate until the maximum speed is reached, 
 * or gets near enough to its target position to start decelerating. 
 * @param motor Motor
 * @param targetPos Target position, in (micro)steps
 */
void motor_goto(motor_t motor, step_t targetPos);
/**
 * @brief Recalculates current motor settings, makes a step if required.
 * 
 * This function should be called on each motor as frequently as possible.
 * 
 * @param motor Motor
 */
void motor_run(motor_t motor);
/**
 * @brief Frees all resources allocated by the motor.
 * 
 * @param motor Motor
 */
void motor_destroy(motor_t motor);

/**
 * @brief Returns position offset when the motor is in tracking mode.
 * 
 * Undefined behaviour when not in tracking mode.
 * 
 * @param m Motor
 * @param t Current time
 * @return int64_t Position offset - how far the motor is from the position it should be at.
 */
int64_t motor_getPosOffset(motor_t m, int64_t t);
#endif