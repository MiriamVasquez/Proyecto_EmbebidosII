/*
 \file
 	 This is the source file for the NVIC device driver for Kinetics k64
 	 It contains some configuration functions and runtime functions
 	 It is implemented using CMSIS Core functions
 \autor Miriam A. Vasquez Juarez, ie728332@iteso.mx
 \date 27/09/2024
 \todo
 	 Pending Interrupts and disable interrumpts are not enable
 */

#include "buttons.h"


void init_button(button_t button){
	const port_pin_config_t input_config = {
	kPORT_PullUp, /*
	Internal pull-up resistor is enabled */
	kPORT_FastSlewRate, /* Fast
	slew rate is configured */
	kPORT_PassiveFilterEnable, /*
	Passive filter is disabled */
	kPORT_OpenDrainDisable, /* Open
	drain is disabled */
	kPORT_HighDriveStrength, /* High
	drive strength is configured */
	kPORT_MuxAsGpio, /* Pin
	is configured as PTA4 */
	kPORT_UnlockRegister /* Pin
	Control Register fields [15:0] are not locked */
	};

	gpio_pin_config_t gpio_input_config = {
			kGPIO_DigitalInput,
			1,
	};

	PORT_SetPinConfig(button.port, button.pin, &input_config);
	PORT_SetPinMux(button.port,button.pin,kPORT_MuxAsGpio);
	GPIO_PinInit(button.gpio, button.pin, &gpio_input_config);
}



void set_button_as_interrupt(button_t button){
	PORT_SetPinInterruptConfig(button.port, button.pin, kPORT_InterruptFallingEdge);
    NVIC_EnableIRQ(button.port == PORTA ? PORTA_IRQn : PORTD_IRQn);

}





