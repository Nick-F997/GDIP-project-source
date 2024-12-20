#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/scb.h" // For vector table offset register

#include "lynxmotion.h"

#include "core/system.h"
#include "core/uart.h"
#include "gpio.h"
#include "sys_timer.h"
#include "adc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BOOTLOADER_SIZE     (0x8000U)


static void loc_vector_setup(void) {
    SCB_VTOR = BOOTLOADER_SIZE; // Offset main Vector Table by size of bootloader so it knows where to look.
}

int main(void)
{
    loc_vector_setup();
    coreSystemSetup();
    LynxMotion_t *robot_main = init_robot(6);
    //setRobotState(STATE_INIT);
    loc_gpio_setup();
    setup_push_button();
    coreUartSetup(115200);
    loc_adc_setup();
    
    setup_emergency_stop_button();

    setRobotState(STATE_TEACH_POS1);
    
    toggle_status_led('r');
    coreSystemDelay(500);
    toggle_status_led('o');
    coreSystemDelay(500);
    toggle_status_led('g');
    coreSystemDelay(500);

    toggle_status_led('o');

    printf("Initialisation Completed!\r\n");
    // uint64_t time_in_move = coreGetTicks();

    while (1)
    {
        state_machine_operations(robot_main);
        printf("\e[1;1H\e[2J");

    }

    return 0;
}
