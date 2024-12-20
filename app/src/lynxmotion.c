/**
 * @file lynxmotion.c
 * @author Nicholas Fairburn (nicholas2.fairburn@live.uwe.ac.uk)
 * @brief Contains function definitions for lynxmotion control structures.
 * @version 0.1
 * @date 2024-11-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "lynxmotion.h"
/** @brief variable for robot state. Exposed by "setRobotState" and "getRobotState" for interrupts.
 *  Should not be used for general access of the variable, as it is also pointed to from LynxMotion_t. Access from there.  */ 
static volatile State RobotState = STATE_INIT;

/**
 * @brief Set the Robot State local volatile variable. Only to be used by interrupts.
 * 
 * @param state 
 */
void setRobotState(State state)
{
    RobotState = state;
}

/**
 * @brief Get the Robot State object, only for use by interrupts
 * 
 * @return State 
 */
State getRobotState(void)
{
    return RobotState;
}


/**
 * @brief Set the Robot State_ object within LynxMotion_t type. Should be used outside of interrupts.
 * 
 * @param arm LynxMotion_t control structure.
 * @param state the desired robot state.
 */
void setRobotState_(LynxMotion_t *arm, State state)
{
    *arm->robot_state = state;
}

/**
 * @brief Get the Robot State_ object within LynxMotion_t type. Should be used outside of interrupts.
 * 
 * @param arm LynxMotion_t control structure.
 * @return State - the current state of state machine.
 */
State getRobotState_(LynxMotion_t *arm)
{
    return *arm->robot_state;
}

/**
 * @brief Function to read adc channel values at sample rate. Calculates scaled value and returns duty cycle.
 * 
 * @param joint the joint to be modified.
 * @return float - the calculated duty cycle.
 */
float read_adc_update_joint_position(LynxMotion_Joint_t *joint)
{
    //joint->adc.current_reading = read_channel_averaged(joint->adc.channel, joint->adc.sample_depth);
    read_channel_smoothed(joint->adc.channel, &joint->adc.current_reading, 2);
    float true_duty = ((float)(joint->adc.current_reading - MIN_POT_VALUE) / (float)(MAX_POT_VALUE - MIN_POT_VALUE)) 
                        * ((MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)) + MIN_DUTY_CYCLE;
    // joint->joint.duty_cycle = true_duty;
    return true_duty;
}

/**
 * @brief Frees a LynxMotion_t structure pointer.
 * 
 * @param arm the structure to be freed.
 */
void free_robot(LynxMotion_t *arm)
{
    for (int joint = 0; joint < arm->num_joints; joint++)
    {
        free(arm->joints[joint]);
        arm->joints[joint] = NULL;
    }
    free(arm->joints);
    arm->joints = NULL;
    free(arm);
    arm = NULL;
}

/**
 * @brief Initialises a pointer to a robot. Creates all joints based on number of joints. 
 * 
 * @param num_joints number of joints to be created.
 * @return LynxMotion_t* - a pointer to the created object. Must be manually freed.
 */
LynxMotion_t *init_robot(uint8_t num_joints)
{
    LynxMotion_t *arm = (LynxMotion_t *)malloc(sizeof(LynxMotion_t));
    if (arm == NULL)
    {
        user_hard_fault_handler("Could not allocate memory for arm struct in: lynxmotion.c/init_robot");
    }
    arm->num_joints = num_joints;
    arm->robot_state = &RobotState;
    arm->joints = (LynxMotion_Joint_t **)malloc(sizeof(LynxMotion_Joint_t *) * arm->num_joints);
    if (arm->joints == NULL)
    {
        user_hard_fault_handler("Could not allocate memory for joint structs in: lynxmotion.c/init_robot");
    }
    
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM4);
    
    for (int joint = 0; joint < arm->num_joints; joint++)
    {
        arm->joints[joint] = (LynxMotion_Joint_t *)malloc(sizeof(LynxMotion_Joint_t));
        arm->joints[joint]->type = (JointType)joint;
        arm->joints[joint]->adc = get_adc_channel(arm->joints[joint]->type); // Setup is done elsewhere as it's really simple
        arm->joints[joint]->positions = (LynxMotion_Joint_Position_t) {.at_position = false, .position1 = 0.0f, .position2 = 0.0f};
        TimerControl_t *target_joint = &arm->joints[joint]->joint; // Create a pointer because that is a horrible line of code
        switch (arm->joints[joint]->type)
        {
            case JOINT_BASE:
            {
                SETUP_JOINT_TIMER(target_joint, 2, 1);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);
                break;
            }
            case JOINT_BASE_UPPER:
            {
                SETUP_JOINT_TIMER(target_joint, 2, 2);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);

                break;
            }
            case JOINT_ELBOW:
            {
                SETUP_JOINT_TIMER(target_joint, 4, 1);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);

                break;
            }
            case JOINT_WRIST_LOWER:
            {
                SETUP_JOINT_TIMER(target_joint, 2, 3);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);
                break;
            }
            case JOINT_WRIST_UPPER:
            {
                SETUP_JOINT_TIMER(target_joint, 3, 2);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);
                break;
            }
            case JOINT_GRIPPER:
            {
                SETUP_JOINT_TIMER(target_joint, 3, 1);
                SET_PID_VALUES(arm->joints[joint], GLOBAL_P, GLOBAL_I, GLOBAL_D);
                break;
            }
        }
    }


    setRobotState_(arm, STATE_TEACH_POS1);
    arm->previous_state = STATE_INIT;
    return arm;

}

/**
 * @brief Moves a joint using PID
 * 
 * @param joint the joint to move
 * @param target_duty the target duty cycle for the PID controller.
 */
static void PIDMoveJoint_old(LynxMotion_Joint_t *joint, float target_duty)
{
    static uint64_t previous_time = 0;
    float error = target_duty - joint->joint.duty_cycle;
    
    uint64_t current_time = coreGetTicks();
    uint64_t dt = current_time - previous_time;
    // Proportional component
    float prop_comp = joint->pid_values.P * error;
    
    // Integral component
    if (joint->pid_values.integrator_sum > 32000)
    {
        joint->pid_values.integrator_sum = 32000;
    }
    if (joint->pid_values.integrator_sum < -32000)
    {
        joint->pid_values.integrator_sum = -32000;
    }

    joint->pid_values.integrator_sum += error;
    float int_comp = joint->pid_values.I * joint->pid_values.integrator_sum  * (float)dt;

    // Derivative Component
    float error_deriv = error - joint->pid_values.previous_error;
    float deriv_component = joint->pid_values.D * (error_deriv / (float)dt);
    joint->pid_values.previous_error = error;

    

    float final_value = prop_comp + int_comp + deriv_component;
    float value_to_write = joint->joint.duty_cycle + final_value;
    
    previous_time = current_time;

    if (value_to_write > MAX_DUTY_CYCLE)
    {
        value_to_write = MAX_DUTY_CYCLE;
    }
    if (value_to_write < MIN_DUTY_CYCLE)
    {
        value_to_write = MIN_DUTY_CYCLE;
    }

    printf("Joint num - %d\tpot value = %f\tpid value = %f\r\n", joint->type, target_duty, final_value);
    corePWMSetDutyCycleStruct(&joint->joint, final_value);

}

/**
 * @brief *UPDATED* Moves a joint using PID
 * 
 * @param joint the joint to move
 * @param target_duty the target duty cycle for the PID controller.
 */
static void PIDMoveJoint(LynxMotion_Joint_t *joint, float target_duty)
{
    static uint64_t previous_time = 0;
    float error = target_duty - joint->joint.duty_cycle;
    
    uint64_t current_time = coreGetTicks();
    uint64_t dt_ticks = current_time - previous_time;
    float dt = (float)dt_ticks / (CLOCKS_PER_SEC / 1000); // Convert ticks to milliseconds
    if (dt < EPSILON) dt = EPSILON; // Prevent division by zero

    // Proportional component
    float prop_comp = joint->pid_values.P * error;
    
    // Integral component
    joint->pid_values.integrator_sum += error * dt;
    if (joint->pid_values.integrator_sum > MAX_INTEGRATOR_SUM)
    {
        joint->pid_values.integrator_sum = MAX_INTEGRATOR_SUM;
    }
    if (joint->pid_values.integrator_sum < -MAX_INTEGRATOR_SUM)
    {
        joint->pid_values.integrator_sum = -MAX_INTEGRATOR_SUM;
    }
    float int_comp = joint->pid_values.I * joint->pid_values.integrator_sum;

    // Derivative component
    float error_deriv = (error - joint->pid_values.previous_error) / dt;
    joint->pid_values.filtered_deriv = (joint->pid_values.filtered_deriv * (1.0 - FILTER_CONSTANT)) + (error_deriv * FILTER_CONSTANT);
    float deriv_comp = joint->pid_values.D * joint->pid_values.filtered_deriv;
    joint->pid_values.previous_error = error;

    // Calculate final PID value
    float final_value = prop_comp + int_comp + deriv_comp;
    
    // Clamp output to valid range
    if (final_value > MAX_DUTY_CYCLE)
    {
        final_value = MAX_DUTY_CYCLE;
    }
    if (final_value < MIN_DUTY_CYCLE)
    {
        final_value = MIN_DUTY_CYCLE;
    }

    // Update duty cycle
    // joint->joint.duty_cycle = final_value;

    // Update previous time
    previous_time = current_time;

    printf("Joint num - %d\tpot value = %.3f\tpid value = %.3f\r\n", joint->type, target_duty, final_value);
    corePWMSetDutyCycleStruct(&joint->joint, final_value);
}

/**
 * @brief Function that manages both teaching states. Condensed into one as there is only one 
 * character change between the two state control functions.
 * 
 * @param arm the arm to modify
 * @param teach current state, sort of redundant but keeps it safer.
 */
static void teach_states(LynxMotion_t *arm, State teach)
{
    printf("------PID Values in state %d -----\r\n", teach);
    printf("==================================\r\n");
    for (int joint = 0; joint < arm->num_joints; joint++)
    {
        float target_dc = read_adc_update_joint_position(arm->joints[joint]);
        // arm->joints[joint]->positions.position1 = arm->joints[joint]->joint.duty_cycle;
        if (teach == STATE_TEACH_POS1) UPDATE_POSITION_STRUCT(arm->joints[joint], target_dc, 1);
        if (teach == STATE_TEACH_POS2) UPDATE_POSITION_STRUCT(arm->joints[joint], target_dc, 2);
        // corePWMSetDutyCycleStruct(&arm->joints[joint]->joint, arm->joints[joint]->joint.duty_cycle);
        PIDMoveJoint(arm->joints[joint], target_dc);
    }
}

/**
 * @brief Helper function to get abs value.
 * 
 * @param numb number to get abs value of.
 * @return float abs value of numb
 */
static float fabs(float numb)
{
    if (numb < 0.0f)
    {
        return -numb;
    }
    else {
        return numb;
    }
}

/**
 * @brief Function to move the arm when in either STATE_MOVING_POS1 or STATE_MOVING_POS2.
 *        Does not move gripper until all other joints are in the correct position.
 * 
 * @param arm Arm structure to move
 * @param state Current state of the robot
 */
static void move_states(LynxMotion_t *arm, State state)
{
    const float MAX_DIFF = 0.5f;
    if (state == STATE_MOVING_POS1)
    {
        uint8_t at_position1 = 0;
        for (int joint = 0; joint < arm->num_joints; joint++)
        {
            if (arm->joints[joint]->type == JOINT_GRIPPER) continue;
            if (!arm->joints[joint]->positions.at_position)
            {
            
                    PIDMoveJoint(arm->joints[joint], arm->joints[joint]->positions.position1);
                    if (fabs(arm->joints[joint]->joint.duty_cycle - arm->joints[joint]->positions.position1) < MAX_DIFF)
                    {
                        arm->joints[joint]->positions.at_position = true;
                    }
            }
            else {
                at_position1++;
            }
            //coreSystemDelay(5);
        }
        if (at_position1 == 5)
        {
            while (!arm->joints[JOINT_GRIPPER]->positions.at_position)
            {
                PIDMoveJoint(arm->joints[JOINT_GRIPPER], arm->joints[JOINT_GRIPPER]->positions.position1);
                if (fabs(arm->joints[JOINT_GRIPPER]->joint.duty_cycle - arm->joints[JOINT_GRIPPER]->positions.position1) < MAX_DIFF)
                {
                    arm->joints[JOINT_GRIPPER]->positions.at_position = true;
                }
            }
            setRobotState_(arm, STATE_MOVING_POS2);
            for (int i = 0; i < arm->num_joints; i++) arm->joints[i]->positions.at_position = false;
            coreSystemDelay(2500);
            return;
        }
    }

    if (state == STATE_MOVING_POS2)
    {
        uint8_t at_position2 = 0;
        for (int joint = 0; joint < arm->num_joints; joint++)
        {
            if (arm->joints[joint]->type == JOINT_GRIPPER) continue;
            if (!arm->joints[joint]->positions.at_position)
            {
                PIDMoveJoint(arm->joints[joint], arm->joints[joint]->positions.position2);
                if (fabs(arm->joints[joint]->joint.duty_cycle - arm->joints[joint]->positions.position2) < MAX_DIFF)
                {
                    arm->joints[joint]->positions.at_position = true;
                }
            }
            else {
                at_position2++;
            }
            // coreSystemDelay(5);

        }
        if (at_position2 == 5)
        {
            while (!arm->joints[JOINT_GRIPPER]->positions.at_position)
            {
                PIDMoveJoint(arm->joints[JOINT_GRIPPER], arm->joints[JOINT_GRIPPER]->positions.position2);
                if (fabs(arm->joints[JOINT_GRIPPER]->joint.duty_cycle - arm->joints[JOINT_GRIPPER]->positions.position2) < MAX_DIFF)
                {
                    arm->joints[JOINT_GRIPPER]->positions.at_position = true;
                }
            }
            setRobotState_(arm, STATE_MOVING_POS1);
            for (int i = 0; i < arm->num_joints; i++) arm->joints[i]->positions.at_position = false;
            coreSystemDelay(2500);
            return;
        }
    }
}


/**
 * @brief The state machine manager. 
 * 
 * @param arm the robot arm to be affected.
 */
void state_machine_operations(LynxMotion_t *arm)
{
    State current_state = getRobotState_(arm);
    switch (current_state)
    {
        case STATE_INIT:
        case STATE_EMERGENCY_STOP:
        {
            break;
        }
        case STATE_MOVING_POS1:
        {
            if (current_state != arm->previous_state)
            {
                printf("Moving state 1\r\n");
            }
            move_states(arm, STATE_MOVING_POS1);
            break;
        }
        case STATE_MOVING_POS2:
        {
            if (current_state != arm->previous_state)
            {
                printf("Moving state 2\r\n");
            }
            move_states(arm, STATE_MOVING_POS2);
            break;
        }
        case STATE_TEACH_POS1:
        {
            if (current_state != arm->previous_state)
            {
                printf("State teach 1\r\n");   
            }
            teach_states(arm, STATE_TEACH_POS1);
            break;
        }
        case STATE_TEACH_POS2:
        {
            if (current_state != arm->previous_state)
            {
                printf("State teach 2\r\n");
            }
            teach_states(arm, STATE_TEACH_POS2);
            break;  
        }
    }
    arm->previous_state = current_state;
}