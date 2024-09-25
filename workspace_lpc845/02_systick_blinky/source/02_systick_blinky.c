#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#define ledazul 1
#define d1 29

int main(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_BootClockFRO24M();

    gpio_pin_config_t out_config = {kGPIO_DigitalOutput, 0};

    GPIO_PortInit(GPIO, 0);
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, ledazul, &out_config);
    GPIO_PinInit(GPIO, 0, d1, &out_config);

    SysTick_Config(SystemCoreClock / 1000);
    while(1);

    return 0 ;
}
void SysTick_Handler(void) {
	static uint16_t i = 0;
	i++;

	if(i % 500 == 0) {
		GPIO_PinWrite(GPIO, 1, ledazul, !GPIO_PinRead(GPIO, 1, ledazul));

	}
	if (i % 1500 == 0 ){
		i=0;
		GPIO_PinWrite(GPIO, 0, d1, !GPIO_PinRead(GPIO, 0, d1));
	}
}
