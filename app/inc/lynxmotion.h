/**
 * @file lynxmotion.h
 * @author Nicholas Fairburn (nicholas2.fairburn@live.uwe.ac.uk)
 * @brief Contains structs, macros, and function prototypes for LynxMotion control.
 * @version 0.1
 * @date 2024-11-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef LYNXMOTION_H_
#define LYNXMOTION_H_

#include <stdio.h>
#include <string.h>

#include "sys_timer.h"
#include "adc.h"
#include "core/uart.h"
#include "core/system.h"
#include "fault-handler.h"

// @brief Minimum Duty cycle for PWM in percentage
#define MIN_DUTY_CYCLE (3.3f)
// @brief Maximum Duty cycle for PWM in percentage
#define MAX_DUTY_CYCLE (13.3f)

// @brief Minimum pot value from ADC channel
#define MIN_POT_VALUE (0)
// @brief Maximum pot value from ADC channel
#define MAX_POT_VALUE (4096)

// #define GLOBAL_P (0.09)
// #define GLOBAL_I (0.0000005)
// #define GLOBAL_D (12.0)

#define GLOBAL_P (0.1)
#define GLOBAL_I (0)
#define GLOBAL_D (0)


/** 
 * @brief Enum used to control the state machine governing movement of robot. Prefixed with 'STATE'.
 * 
 */
typedef enum State {
    STATE_INIT,
    STATE_MOVING_POS1,
    STATE_MOVING_POS2,
    STATE_TEACH_POS1,
    STATE_TEACH_POS2,
    STATE_EMERGENCY_STOP,
} State;

/**
 * @brief Enum to differentiate each joint of the robot. Prefixed with 'JOINT'.
 * 
 */
typedef enum {
    JOINT_BASE,
    JOINT_BASE_UPPER,
    JOINT_ELBOW,
    JOINT_WRIST_LOWER,
    JOINT_WRIST_UPPER,
    JOINT_GRIPPER,
} JointType;

typedef struct PIDControl_t
{
    float P;
    float I;
    float D;
    float integrator_sum;
    float previous_error;
} PIDControl_t;

/**
 * @brief Basic structure that contains robot joint positions for each state. 
 * 
 */
typedef struct LynxMotion_Joint_Position_t
{
    float position1;
    float position2;
    bool at_position;
} LynxMotion_Joint_Position_t;

/**
 * @brief Structure that contains information and control for each joint.
 *
 * 
 */
typedef struct LynxMotion_Joint_t
{
    ADCControl_t adc;
    TimerControl_t joint;
    JointType type;
    LynxMotion_Joint_Position_t positions;
    PIDControl_t pid_values;
} LynxMotion_Joint_t;


/**
 * @brief Struct that contains all joint information, as well as state machine information. 'top-level' strucutre.
 * 
 */
typedef struct LynxMotion_t
{
    LynxMotion_Joint_t **joints;
    uint8_t num_joints;
    volatile State *robot_state;
    State previous_state;
} LynxMotion_t;

/// @brief helper macro to make timer set up easier.
#define SETUP_JOINT_TIMER(joint, rcc, oc) (coreTimerSetupReturns(joint, RCC_TIM##rcc, TIM##rcc, 84 + 1, 20000 + 1, TIM_OC##oc))

/// @brief helper macro to update position structs
#define UPDATE_POSITION_STRUCT(joint_t, duty_cycle, pos) (joint_t->positions.position##pos = duty_cycle)

#define SET_PID_VALUES(joint_t, PP, II, DD) (joint_t->pid_values = (PIDControl_t) {.P = PP, .I = II, .D = DD, .integrator_sum = 0.0f, .previous_error = 0.0f})

void setRobotState(State state);
State getRobotState(void);
void setRobotState_(LynxMotion_t *arm, State state);
State getRobotState_(LynxMotion_t *arm);
LynxMotion_t *init_robot(uint8_t num_joints);
void state_machine_operations(LynxMotion_t *arm);
void free_robot(LynxMotion_t *arm);
float read_adc_update_joint_position(LynxMotion_Joint_t *joint);

#endif