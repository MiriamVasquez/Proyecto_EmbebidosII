/*
 \file
 	 This is the source file for the NVIC device driver for Kinetics k64
 	 It contains some configuration functions and runtime functions
 	 It is implemented using CMSIS Core functions
 \autor Miriam A. Vasquez Juarez, ie728332@iteso.mx
 \date 07/09/2024
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "config.h"
#include "buttons.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#define SetPin_GPIO(PORT,PIN) 			({PORT->PCR[PIN] |= GPIO_MUX_MASK;})
#define SET_PIN_AS_INPUT(GPIO,MASK) 	({GPIO->PDDR &= GPIO_FIT_REG(~(MASK));})
#define SET_PIN_AS_OUTPUT(GPIO,MASK) 	({GPIO->PDDR |= MASK;})


typedef struct{
	uint8_t	pin;
	GPIO_Type*	gpio;
	PORT_Type*	port;
}gpio_config_t;

typedef enum{
	GPIO_A,
	GPIO_B,
	GPIO_C,
	GPIO_D,
	GPIO_E
} gpio_name_t;

typedef enum{
	PORT_A,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E
} port_t;

typedef struct
{
	uint8_t flag_port_a : 1;
	uint8_t flag_port_b : 1;
	uint8_t flag_port_c : 1;
	uint8_t flag_port_d : 1;
	uint8_t flag_port_e : 1;
} gpio_interrupt_flags_t;

#define LED_PORT (PORTA)
#define LED_GPIO (GPIOA)
#define LED_PIN  (11U)

void gpio_init_as_output(void);
void gpio_init_buttons(void);
void set_gpiocISRflags(uint32_t value);
uint32_t get_gpiocISRflags(void);
void set_gpioaISRflag(uint32_t value);
uint8_t get_gpioaISRflag(void);
void set_gpiobISRflags(uint32_t value);
uint32_t get_gpiobISRflags(void);
void set_gpiodISRflags(uint32_t value);
uint32_t get_gpiodISRflags(void);
void GPIO_clear_irq_status(gpio_name_t gpio);
//void PORTA_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTD_IRQHandler(void);//void PORTD_IRQHandler(void);
void PORTC_IRQHandler(void);
void GPIO_callback_init(gpio_name_t gpio, void (*handler)(uint32_t flags));
uint8_t GPIO_get_irq_status(gpio_name_t gpio);
void clearFlags(void);
void GPIO_init_output(gpio_config_t gpio);

#endif /* GPIO_H_ */

