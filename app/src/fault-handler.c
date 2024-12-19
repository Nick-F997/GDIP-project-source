/**
 * @file fault-handler.c
 * @author Nicholas Fairburn (nicholas2.fairburn@live.uwe.ac.uk)
 * @brief Contains the hard fault handler.
 * @version 0.1
 * @date 2024-12-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "fault-handler.h"

/**
 * @brief Hard fault handling. Also used for emergency stop.
 * 
 * @param message what has caused the error.
 */
void user_hard_fault_handler(char *message)
{
    printf("\e[1;1H\e[2J");
    uint64_t time_of_fault = coreGetTicks();
    uint64_t seconds_time_of_fault = (uint64_t)(time_of_fault / 1000);
    printf("Error at %llds: %s. Please restart to recover.\r\n", seconds_time_of_fault, message);
    while (true)
    {
        // Hang forever.
    }
}