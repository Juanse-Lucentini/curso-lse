#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"


#define LED_RED	2
#define USER_BUTTON 4

/*
 * @brief   Application entry point.
*/
int main(void) {

    BOARD_InitBootClocks();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();


    gpio_pin_config_t config_led = { kGPIO_DigitalOutput, 1 };
    gpio_pin_config_t config_button = { kGPIO_DigitalInput, 0};

    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, LED_RED, &config_led);

    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, USER_BUTTON, &config_button);

    while(1) {
    	if(GPIO_PinRead(GPIO, 0, USER_BUTTON)){
    		GPIO_PinWrite(GPIO, 1, LED_RED, 1);
    		for(uint32_t i = 0; i < 1; i++);
    	}
    	else {
    		GPIO_PinWrite(GPIO, 1, LED_RED, 0);
    		for(uint32_t i = 0; i < 1; i++);
    	}

    }
    return 0;
}
