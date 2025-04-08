/*
 * gpio.c
 *
 *  Created on: Sep 26, 2024
 *      Author: Jorge Leautad
 */

#include "gpio.h"

volatile static gpio_interrupt_flags_t g_intr_status_flag = {FALSE};

static void (*gpio_C_callback)(uint32_t flags) = FALSE;
static void (*gpio_A_callback)(uint32_t flags) = FALSE;
static void (*gpio_B_callback)(uint32_t flags) = FALSE;
static void (*gpio_D_callback)(uint32_t flags) = FALSE;

volatile static uint32_t gpiocISRflags = FALSE;
volatile static uint8_t gpioaISRflag = FALSE;
volatile static uint32_t gpiobISRflags = FALSE;
volatile static uint32_t gpiodISRflags = FALSE;

void gpio_init_as_output(void){
    SetPin_GPIO(RED_LED_PORT, RED_LED_PIN);
    SetPin_GPIO(BLUE_LED_PORT, BLUE_LED_PIN);
    SetPin_GPIO(GREEN_LED_PORT, GREEN_LED_PIN);

    SET_PIN_AS_OUTPUT(RED_LED_GPIO,RED_LED_MASK);
    SET_PIN_AS_OUTPUT(BLUE_LED_GPIO,BLUE_LED_MASK);
    SET_PIN_AS_OUTPUT(GREEN_LED_GPIO,GREEN_LED_MASK);
}

void gpio_init_buttons(void) {
    CLOCK_EnableClock(kCLOCK_PortA); // Para SW2
    CLOCK_EnableClock(kCLOCK_PortD); // Para SW3

    PORT_SetPinMux(SW2_PORT, SW2_PIN, kPORT_MuxAsGpio); // SW2
    PORT_SetPinMux(SW3_PORT, SW3_PIN, kPORT_MuxAsGpio); // SW3

    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalInput, 0 // 0 para indicar que no se usa un estado inicial específico
    };

    GPIO_PinInit(SW2_GPIO, SW2_PIN, &gpio_config); // Inicializa SW2 como entrada
    GPIO_PinInit(SW3_GPIO, SW3_PIN, &gpio_config); // Inicializa SW3 como entrada
}

void set_gpiocISRflags(uint32_t value){
	gpiocISRflags = value;
}

uint32_t get_gpiocISRflags(void){
	return gpiocISRflags;
}

void set_gpioaISRflag(uint32_t value){
	gpioaISRflag = value;
}

uint8_t get_gpioaISRflag(void){
	return gpioaISRflag;
}

void set_gpiobISRflags(uint32_t value){
	gpiobISRflags = value;
}

uint32_t get_gpiobISRflags(void){
	return gpiobISRflags;
}

void set_gpiodISRflags(uint32_t value){
	gpiodISRflags = value;
}

uint32_t get_gpiodISRflags(void){
	return gpiodISRflags;
}

void GPIO_callback_init(gpio_name_t gpio, void (*handler)(uint32_t flags)){
	if(GPIO_A == gpio){
		gpio_A_callback = handler;
	}
	else if (GPIO_C == gpio){
		gpio_C_callback = handler;
	}
	else if (GPIO_B == gpio){
		gpio_B_callback = handler;
	}
	else if (GPIO_D == gpio){
		gpio_D_callback = handler;
	}
}

void GPIO_init_output(gpio_config_t gpio){
	gpio_pin_config_t gpio_config = {
			kGPIO_DigitalOutput,
			1,
	};

	PORT_SetPinMux(gpio.port,gpio.pin,kPORT_MuxAsGpio);
	GPIO_PinInit(gpio.gpio, gpio.pin, &gpio_config);
}

//asi se llama en el MCUX la funcion que tiene que atender una interrupcion
void PORTC_IRQHandler(void){
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOC);
	set_gpiocISRflags(GPIO_PortGetInterruptFlags(GPIOC));

	if(gpio_C_callback)
	{
		gpio_C_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOC, 0xFFFFFFFF);
}
void PORTD_IRQHandler(void){
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOD);
	set_gpiodISRflags(GPIO_PortGetInterruptFlags(GPIOD));

	if(gpio_D_callback)
	{
		gpio_D_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOD, 0xFFFFFFFF);
}

void PORTB_IRQHandler(void){
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOB);
	set_gpiobISRflags(GPIO_PortGetInterruptFlags(GPIOB));

	if(gpio_B_callback)
	{
		gpio_B_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOB, 0xFFFFFFFF);
}

void PORTA_IRQHandler(void){
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOA);
	set_gpioaISRflag(GPIO_PortGetInterruptFlags(GPIOA));

	if(gpio_A_callback)
	{
		gpio_A_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOA, 0xFFFFFFFF);
}


void GPIO_clear_irq_status(gpio_name_t gpio)
{
	if(GPIO_A == gpio){
		g_intr_status_flag.flag_port_a = false;
	}
	else{
		g_intr_status_flag.flag_port_c = false;
	}
}

uint8_t GPIO_get_irq_status(gpio_name_t gpio)
{
	uint8_t status = 0;

	if(GPIO_A == gpio){
		status = g_intr_status_flag.flag_port_a;
	}
	else{
		status = g_intr_status_flag.flag_port_c;
	}

	return(status);
}

void clearFlags(void){
	set_gpiodISRflags(FALSE);
	set_gpioaISRflag(FALSE);
	set_gpiobISRflags(FALSE);
	set_gpiocISRflags(FALSE);
}

