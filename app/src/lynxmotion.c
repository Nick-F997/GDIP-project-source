#include "lynxmotion.h"

static volatile State RobotState = STATE_INIT;

void setRobotState(State state)
{
    RobotState = state;
}

State getRobotState(void)
{
    return RobotState;
}