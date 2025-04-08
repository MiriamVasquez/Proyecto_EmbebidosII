/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_adc16.h"

#include "LCD_nokia.h"
#include "buttons.h"
#include "gpio.h"
#include "NVIC.h"
#include "LCD_nokia_images.h"
#include "SPI.h"
#include "Delay.h"
#include "nokia_draw.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define GrapNumb_PRIORITY   (configMAX_PRIORITIES - 2)
#define X_INCREMENT 1
#define HM_HEART_ADC16_BASE        ADC1
#define HM_ADC16_CHANNEL_GROUP     0U
#define HM_ADC16_USER_CHANNEL      23U
#define HM_ADC16_IRQ_HANDLER_FUNC ADC1_IRQHandler

typedef struct{
    uint8_t x;
    uint8_t y;
}point_str;

typedef struct{
    uint8_t convSource;
    uint16_t data;
}adcConv_str;

TimerHandle_t SendFBTimer;
TimerHandle_t ADCConversionTimer;
adc16_channel_config_t adc16ChannelConfigStruct;
QueueHandle_t TimeScaleMailbox;
QueueHandle_t AdcConversionQueue;
QueueHandle_t PointQueue;
QueueHandle_t NumberQueue;
QueueHandle_t alarmQueue;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void LCDprint_thread(void *pvParameters);
static void GraphProcess_thread(void *pvParameters);
static void NumberProcess_thread(void *pvParameters);
static void Alarm_thread(void *pvParameters);

uint16_t up_limit = 200;
uint16_t inferior_limit = 50;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
void ScreenInit(){
	init_SPI();
    LCD_nokia_init(); /*! Configuration function for the LCD */
    LCD_nokia_clear();
}

void ADCInit(){
    adc16_config_t adc16ConfigStruct;

    NVIC_SetPriority(ADC1_IRQn,3);
    EnableIRQ(ADC1_IRQn);

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(ADC1, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC1, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC1)){
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else{
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif
    adc16ChannelConfigStruct.channelNumber = HM_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}


void HM_ADC16_IRQ_HANDLER_FUNC(void){
    uint16_t conversionVal = 0;
    adcConv_str AdcConv;
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;

    /* Read conversion result to clear the conversion completed flag. */
    AdcConv.data = ADC16_GetChannelConversionValue(HM_HEART_ADC16_BASE, HM_ADC16_CHANNEL_GROUP);
    /*Send conversion value to the ADC queue*/
    xQueueSendFromISR(AdcConversionQueue,&AdcConv,&xHigherPriorityTaskWoken);
    /*Restore flag for the ISR-Thread safe purpose*/
    xHigherPriorityTaskWoken = pdFALSE;
    /*Send conversion value to the ADC queue again. Need same value twice since is consumed by two different threads*/
    xQueueSendFromISR(AdcConversionQueue,&AdcConv,&xHigherPriorityTaskWoken);

    SDK_ISR_EXIT_BARRIER;
}

void time_scale(uint32_t flags){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t x_increment = 1;

	if(x_increment<10){
		x_increment += 5;
	}else{
		x_increment = 1;
	}
	xQueueOverwriteFromISR(TimeScaleMailbox,&x_increment,&xHigherPriorityTaskWoken);
}

void SendFBCallback(TimerHandle_t ARHandle){
    LCD_nokia_sent_FrameBuffer();
}

void ADCConversionCallback(TimerHandle_t ARHandle){
    /*Request ADC conversion*/
    ADC16_SetChannelConfig(HM_HEART_ADC16_BASE, HM_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}

int main(void){
    uint8_t init_time_scale = 1;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    ScreenInit();
    ADCInit();

    init_button(sw3);
    NVIC_set_basepri_threshold(PRIORITY_10);
    NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_4);
    NVIC_global_enable_interrupts;

    set_button_as_interrupt(sw3);
    GPIO_callback_init(GPIO_A, time_scale);

    AdcConversionQueue = xQueueCreate(5,sizeof(adcConv_str));
    PointQueue = xQueueCreate(5,sizeof(uint16_t));
    NumberQueue = xQueueCreate(5,sizeof(uint16_t));
    TimeScaleMailbox = xQueueCreate(1,sizeof(uint8_t));
    alarmQueue = xQueueCreate(1,sizeof(uint8_t));

    SendFBTimer = xTimerCreate(
            "WriteFB", /*Timer's Name*/
            pdMS_TO_TICKS(33), /* Expiration time = 33ms*/
            pdTRUE, /*Auto-reload mode*/
            0, /*Timer's ID. Not used*/
            SendFBCallback); /*Auto-Reload notification function. Called at expiration time*/

    ADCConversionTimer = xTimerCreate(
            "ADCConversion", /*Timer's Name*/
            pdMS_TO_TICKS(100), /* Expiration time = 10ms*/
            pdTRUE, /*Auto-reload mode*/
            0, /*Timer's ID. Not used*/
            ADCConversionCallback); /*Auto-Reload notification function. Called at expiration time*/
    /*Start FrameBuffer update timer*/
    xTimerStart(SendFBTimer,0);
    /*Start ADC measurement timer*/
    xTimerStart(ADCConversionTimer,0);
    /*Publish time-scale init value*/
    xQueueOverwrite(TimeScaleMailbox,&init_time_scale);

    if (xTaskCreate(LCDprint_thread, "LCDprint_thread", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY, NULL) !=pdPASS){
        PRINTF("LCDprint_thread creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(GraphProcess_thread, "GraphProcess_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
        PRINTF("LCDprint_thread creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(NumberProcess_thread, "NumberProcess_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
        PRINTF("NumberProcess_thread creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(Alarm_thread, "Alarm_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
        PRINTF("Alarm_thread creation failed!.\r\n");
        while (1);
        }
    vTaskStartScheduler();
    for (;;);
}


static void Alarm_thread(void *pvParameters){
    uint16_t alarm_value;
    uint8_t set_alarm;
    static uint16_t count_ss = 0;
    static uint16_t count_up_limit = 0;
    static uint16_t count_inferior_limit = 0;

    for (;;){
        /*Wait for the ADC conversion to be avaliable*/
        if(xQueueReceive(NumberQueue,&alarm_value ,pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
        	if (alarm_value < inferior_limit){
          		count_inferior_limit++;
          		count_up_limit = 0;
          		count_ss = 0;
        	}
        	else if (alarm_value > up_limit){
        	    count_up_limit++;
           		count_inferior_limit = 0;
           		count_ss = 0;
          	}
        	else{
        		count_inferior_limit = 0;
        		count_up_limit = 0;
        		count_ss ++;
        		if(count_ss > 20){
					set_alarm = 0;
					xQueueSend(alarmQueue,&set_alarm,portMAX_DELAY);
        		}
        	}

        	if(count_up_limit > 50 ||count_inferior_limit > 50 ){
        		set_alarm = 1;
        		xQueueSend(alarmQueue,&set_alarm,portMAX_DELAY);
        	}
        }
    }
}

static void LCDprint_thread(void *pvParameters){
    point_str point_a = {0};
    point_str point_b = {0};
    uint16_t ypointGraph;
    uint16_t ypointNumber;
    uint8_t x_increment;
    uint8_t onedigit = 0;
    uint8_t secdigit = 0;
    uint8_t thrdigit = 0;
    static uint8_t set_alarm = 0;

    for (;;){
        /*Received ADC processed value to the graphic scale*/
        if(xQueueReceive(PointQueue,&ypointGraph, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
            /*Read the time-scale currently configured*/
            if(xQueuePeek(TimeScaleMailbox,&x_increment,pdMS_TO_TICKS(5))){
                /*Get next point to be graphed*/
                point_a = point_b;
                point_b.y = ypointGraph;
                /*Check if next point is range*/
                if(point_b.x<84){
                    point_b.x += x_increment;
                }
                /*If not in range, clear the graph zone and start in 0*/
                else{
                    point_b.x = 0;
                    LCD_nokia_clear_range_FrameBuffer(0,3,252);
                }
                /*Draw a line between the old and new points and draw 50 points to complete the line*/
                drawline(point_a.x,point_a.y,point_b.x,point_b.y,50);
            }
        }
        /*Received ADC processed value for the numeric scale*/
        if(xQueueReceive(NumberQueue,&ypointNumber, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
            LCD_nokia_clear_range_FrameBuffer(0,1,20);
            onedigit = ((ypointNumber/100)%10)+0x30;
            secdigit = ((ypointNumber/10)%10)+0x30;
            thrdigit = (ypointNumber%10)+0x30;
            LCD_nokia_write_char_xy_FB(0,1,onedigit);
            LCD_nokia_write_char_xy_FB(5,1,secdigit);
            LCD_nokia_write_char_xy_FB(10,1,'.');
            LCD_nokia_write_char_xy_FB(15,1,thrdigit);
            LCD_nokia_write_char_xy_FB(20,1,' ');
            LCD_nokia_write_string_xy_FB(25,1,"mv");
            taskYIELD();
        }
        if(xQueueReceive(alarmQueue,&set_alarm, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
        	if (set_alarm){
        		LCD_nokia_write_string_xy_FB(1,2,"ALARM");
        	}
        	else{
        		LCD_nokia_write_string_xy_FB(1,2,"     ");
        	}
        }
    }
}


static void GraphProcess_thread(void *pvParameters){
    adcConv_str adcConvVal;
    uint16_t processedVal;
    for (;;){
        /*Wait for the ADC conversion to be avaliable*/
        if(xQueueReceive(AdcConversionQueue,&adcConvVal,portMAX_DELAY) != errQUEUE_EMPTY){
            /*Process the raw adc value to be in range from 0 to 43*/
            processedVal = ((adcConvVal.data * 6)/1025);
            xQueueSend(PointQueue,&processedVal,portMAX_DELAY);
            taskYIELD();
        }
    }
}

static void NumberProcess_thread(void *pvParameters){
    adcConv_str adcConvVal;
    uint16_t processedVal;
    for (;;){
        /*Wait for the ADC conversion to be avaliable*/
        if(xQueueReceive(AdcConversionQueue,&adcConvVal,portMAX_DELAY) != errQUEUE_EMPTY){
            /*Process the raw adc value to be in range from 0 to 327 (3.27mv)*/
            processedVal = ((adcConvVal.data * 8)/100);
            xQueueSend(NumberQueue,&processedVal,portMAX_DELAY);
            //send twice to read again
            xQueueSend(NumberQueue,&processedVal,portMAX_DELAY);
            taskYIELD();
        }
    }
}
