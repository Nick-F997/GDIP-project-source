/**
 * @file fault-handler.h
 * @author Nicholas Fairburn (nicholas2.fairburn@live.uwe.ac.uk)
 * @brief Prototypes the hard fault handler
 * @version 0.1
 * @date 2024-12-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef FAULT_HANDLER_H_
#define FAULT_HANDLER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/uart.h"
#include "core/system.h"
#include "core/system.h"

void user_hard_fault_handler(char *message);

#endif