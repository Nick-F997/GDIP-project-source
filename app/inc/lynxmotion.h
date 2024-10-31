#ifndef LYNXMOTION_H_
#define LYNXMOTION_H_

#include "sys_timer.h"

// enum LynxArmStatus {
//     STATE_IDLE,
//     STATE_TEACHING,
//     STATE_AUTONOMOUS,
// };

// typedef struct PIDStruct {
//     float P;
//     float I;
//     float D;
//     float error;
// } PIDStruct;

// typedef struct LynxJoint_t {
//     float current_angle;
//     float target_angle;
//     TimerControl_t motor_control;
//     PIDStruct pid_coeffs; // Currently unused.
// } LynxJoint_t;

// typedef struct LynxMotion_t {
//     enum LynxArmStatus status;
//     LynxJoint_t *joints;
//     size_t num_joints;
// } LynxMotion_t;


typedef enum {
    STATE_INIT,
    STATE_MOVING_POS1,
    STATE_MOVING_POS2,
    STATE_TEACH_POS1,
    STATE_TEACH_POS2,
    STATE_EMERGENCY_STOP,
} State;

void setRobotState(State state);
State getRobotState(void);

#endif