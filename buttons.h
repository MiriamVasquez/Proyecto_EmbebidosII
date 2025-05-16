/*
 \file

 \autor Miriam A. Vasquez Juarez, ie728332@iteso.mx
 \date 27/09/2024
 \todo
 	 Pending Interrupts and disable interrumpts are not enable
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "fsl_port.h"
#include "fsl_gpio.h"

#define SW2_GPIO				(GPIOD)
#define SW3_GPIO				(GPIOA)
#define SW2_PORT				(PORTD)
#define SW3_PORT				(PORTA)
#define	SW2_PIN					(11)
#define	SW3_PIN					(10)
#define SW2_MASK				(1 << SW2_PIN)
#define SW3_MASK				(1 << SW3_PIN)


#define PINH_GPIO				(GPIOC)
#define PINL_GPIO				(GPIOC)
#define PINH_PORT				(PORTC)
#define PINL_PORT				(PORTC)
#define	PINH_PIN					(4)
#define	PINL_PIN					(3)
#define PINH_MASK				(1 << PINH_PIN)
#define PINL_MASK				(1 << PINL_PIN)

typedef struct{
	uint8_t	pin;
	uint32_t mask;
	PORT_Type*	port;
	GPIO_Type*	gpio;
}button_t;


void init_button(button_t button);
void set_button_as_interrupt(button_t button);

static const button_t sw2 = {SW2_PIN,
								SW2_MASK,
								SW2_PORT,
								SW2_GPIO};

static const button_t sw3 = {SW3_PIN,
								SW3_MASK,
								SW3_PORT,
								SW3_GPIO};

static const button_t pinH = {PINH_PIN,
								PINH_MASK,
								PINH_PORT,
								PINH_GPIO};

static const button_t pinL = {PINL_PIN,
								PINL_MASK,
								PINL_PORT,
								PINL_GPIO};



#endif /* BUTTONS_H_ */
