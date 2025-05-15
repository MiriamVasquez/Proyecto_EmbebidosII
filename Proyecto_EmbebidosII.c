/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Proyecto_EmbebidosII.c
 * @brief   Application entry point (modificado).
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"

#include "LCD_nokia.h"
#include "buttons.h"
#include "gpio.h"
#include "NVIC.h"
#include "LCD_nokia_images.h"
#include "SPI.h"
#include "Delay.h"
#include "nokia_draw.h"

#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define GrapNumb_PRIORITY   (configMAX_PRIORITIES - 2)

typedef struct{
	uint8_t x;
	uint8_t y;
} point_str;

QueueHandle_t NumberQueue;
QueueHandle_t PointQueue;
QueueHandle_t alarmQueue;
QueueHandle_t limitsQueue;
QueueHandle_t UartRxQueue;
QueueHandle_t TimeScaleMailbox;
QueueHandle_t ScreenSelectorMailbox;
SemaphoreHandle_t xUartMutex;
SemaphoreHandle_t xLCDMutex;
TimerHandle_t SendFBTimer;

static void LCDprint_thread(void *pvParameters);
static void Alarm_thread(void *pvParameters);
static void Terminal_thread(void *pvParameters);

void ScreenInit() {
	init_SPI();
	LCD_nokia_init();
	LCD_nokia_clear();
}

void time_scale(uint32_t flags){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t x_increment = 1;
	if(x_increment < 10){
		x_increment += 5;
	} else {
		x_increment = 1;
	}
	xQueueOverwriteFromISR(TimeScaleMailbox, &x_increment, &xHigherPriorityTaskWoken);
}

void screen_selector(uint32_t flags) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t screen_selection = 1;
	if(screen_selection < 3){
		screen_selection++;
	} else {
		screen_selection = 1;
	}
	xQueueOverwriteFromISR(ScreenSelectorMailbox, &screen_selection, &xHigherPriorityTaskWoken);
}

void SendFBCallback(TimerHandle_t ARHandle){
	LCD_nokia_sent_FrameBuffer();
}

void UART0_RX_TX_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	char c;
	if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART0)) {
		c = UART_ReadByte(UART0);
		xQueueSendFromISR(UartRxQueue, &c, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

int main(void) {
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_SetSimSafeDivs();

	uint8_t init_time_scale = 1;
	uint8_t init_screen = 1;

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	EnableIRQ(UART0_RX_TX_IRQn);
	NVIC_SetPriority(UART0_RX_TX_IRQn, 5);

	UartRxQueue = xQueueCreate(32, sizeof(char));
	xUartMutex = xSemaphoreCreateMutex();
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable);

	ScreenInit();

	init_button(sw3);
	init_button(sw2);
	gpio_init_as_output();

	NVIC_set_basepri_threshold(PRIORITY_10);
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_4);
	NVIC_enable_interrupt_and_priotity(PORTD_IRQ, PRIORITY_4);
	NVIC_global_enable_interrupts;

	set_button_as_interrupt(sw3);
	set_button_as_interrupt(sw2);

	GPIO_callback_init(GPIO_A, time_scale);
	GPIO_callback_init(GPIO_D, screen_selector);

	PointQueue = xQueueCreate(5, sizeof(uint16_t));
	NumberQueue = xQueueCreate(5, sizeof(uint16_t));
	alarmQueue = xQueueCreate(1, sizeof(uint8_t));
	limitsQueue = xQueueCreate(4, sizeof(uint16_t));

	TimeScaleMailbox = xQueueCreate(1, sizeof(uint8_t));
	ScreenSelectorMailbox = xQueueCreate(1, sizeof(uint8_t));

	SendFBTimer = xTimerCreate("WriteFB", pdMS_TO_TICKS(33), pdTRUE, 0, SendFBCallback);
	xTimerStart(SendFBTimer, 0);

	xQueueOverwrite(TimeScaleMailbox, &init_time_scale);
	xQueueOverwrite(ScreenSelectorMailbox, &init_screen);

	xLCDMutex = xSemaphoreCreateMutex();

	if (xTaskCreate(LCDprint_thread, "LCDprint_thread", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY, NULL) != pdPASS) {
		PRINTF("LCDprint_thread creation failed!.\r\n");
		while (1);
	}

	if (xTaskCreate(Alarm_thread, "Alarm_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) != pdPASS) {
		PRINTF("Alarm_thread creation failed!.\r\n");
		while (1);
	}

	if (xTaskCreate(Terminal_thread, "Terminal_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) != pdPASS) {
		PRINTF("Terminal_thread creation failed!.\r\n");
		while (1);
	}

	led1_off();
	led2_off();

	vTaskStartScheduler();
	for(;;);
}

// Implementaciones mínimas de LCDprint_thread, Alarm_thread, Terminal_thread aquí...
