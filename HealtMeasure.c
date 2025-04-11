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

#define HM_HEART_ADC1_BASE        ADC1
#define HM_ADC1_CHANNEL_GROUP     0U
#define HM_ADC1_USER_CHANNEL      23U
#define HM_ADC1_IRQ_HANDLER_FUNC 		ADC1_IRQHandler

#define HM_HEART_ADC16_BASE        ADC0
#define HM_ADC16_CHANNEL_GROUP     0U
#define HM_ADC16_USER_CHANNEL      16U
#define HM_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

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
TimerHandle_t ADCConversionTempTimer;
adc16_channel_config_t adc16ChannelConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct2;
QueueHandle_t TimeScaleMailbox;
QueueHandle_t ScreenSelectorMailbox;
QueueHandle_t AdcConversionQueue;
QueueHandle_t AdcConversionTempQueue;
QueueHandle_t PointQueue;
QueueHandle_t PointTempQueue;
QueueHandle_t NumberQueue;
QueueHandle_t NumberTempQueue;
QueueHandle_t alarmQueue;
QueueHandle_t alarmTempQueue;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void LCDprint_thread(void *pvParameters);
static void GraphProcess_thread(void *pvParameters);
static void NumberProcess_thread(void *pvParameters);
static void NumberProcessTemp_thread(void *pvParameters);
static void Alarm_thread(void *pvParameters);


uint16_t up_limit = 200;
uint16_t inferior_limit = 50;
uint16_t upTemp_limit = 380;
uint16_t inferiorTemp_limit = 345;

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

void ADCInitADC1(){
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
    adc16ChannelConfigStruct.channelNumber = HM_ADC1_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}
void ADCInitADC0(){
    adc16_config_t adc16ConfigStruct;

    NVIC_SetPriority(ADC0_IRQn,3);
    EnableIRQ(ADC0_IRQn);

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(ADC0, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC0))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif
    adc16ChannelConfigStruct2.channelNumber = HM_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct2.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct2.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}
void HM_ADC16_IRQ_HANDLER_FUNC(void){
    adcConv_str AdcConv;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    AdcConv.data = ADC16_GetChannelConversionValue(HM_HEART_ADC16_BASE, HM_ADC16_CHANNEL_GROUP);
    xQueueSendFromISR(AdcConversionTempQueue,&AdcConv,&xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(AdcConversionTempQueue,&AdcConv,&xHigherPriorityTaskWoken);

    SDK_ISR_EXIT_BARRIER;
}

void HM_ADC1_IRQ_HANDLER_FUNC(void){
    adcConv_str AdcConv;
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;

    /* Read conversion result to clear the conversion completed flag. */
    AdcConv.data = ADC16_GetChannelConversionValue(HM_HEART_ADC1_BASE, HM_ADC1_CHANNEL_GROUP);
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

void screen_selector(uint32_t flags){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t screen_selection = 1;

	if(screen_selection < 3){
		screen_selection++;
	}else{
		screen_selection = 1;
	}
	xQueueOverwriteFromISR(ScreenSelectorMailbox,&screen_selection,&xHigherPriorityTaskWoken);
}

void SendFBCallback(TimerHandle_t ARHandle){
    LCD_nokia_sent_FrameBuffer();
}

void ADCConversionCallback(TimerHandle_t ARHandle){
    /*Request ADC conversion*/
    ADC16_SetChannelConfig(HM_HEART_ADC1_BASE, HM_ADC1_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}
void ADCConversionTempCallback(TimerHandle_t ARHandle){
    ADC16_SetChannelConfig(HM_HEART_ADC16_BASE,HM_ADC16_CHANNEL_GROUP , &adc16ChannelConfigStruct2);
}
int main(void){
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_SetSimSafeDivs();
    uint8_t init_time_scale = 1;
    uint8_t init_screen = 2;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    ScreenInit();
    ADCInitADC1();
    ADCInitADC0();

    init_button(sw3);
    init_button(sw2);
    gpio_init_as_output();

    NVIC_set_basepri_threshold(PRIORITY_10);
    NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_4);
    NVIC_enable_interrupt_and_priotity(PORTD_IRQ,PRIORITY_4);
    NVIC_global_enable_interrupts;

    set_button_as_interrupt(sw3);
    set_button_as_interrupt(sw2);

    GPIO_callback_init(GPIO_A, time_scale);
    GPIO_callback_init(GPIO_D, screen_selector);

    AdcConversionQueue = xQueueCreate(5,sizeof(adcConv_str));
    AdcConversionTempQueue = xQueueCreate(5,sizeof(adcConv_str));
    PointQueue = xQueueCreate(5,sizeof(uint16_t));
    PointTempQueue = xQueueCreate(5,sizeof(uint16_t));
    NumberQueue = xQueueCreate(5,sizeof(uint16_t));
    NumberTempQueue = xQueueCreate(5,sizeof(uint16_t));
    TimeScaleMailbox = xQueueCreate(1,sizeof(uint8_t));
    ScreenSelectorMailbox = xQueueCreate(1,sizeof(uint8_t));
    alarmQueue = xQueueCreate(1,sizeof(uint8_t));
    alarmTempQueue = xQueueCreate(1,sizeof(uint8_t));

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
    ADCConversionTempTimer = xTimerCreate("ADCConversionTemp",pdMS_TO_TICKS(100),pdTRUE,0, ADCConversionTempCallback);
    /*Start FrameBuffer update timer*/
    xTimerStart(SendFBTimer,0);
    /*Start ADC measurement timer*/
    xTimerStart(ADCConversionTimer,0);
    xTimerStart(ADCConversionTempTimer,0);
    /*Publish time-scale init value*/
    xQueueOverwrite(TimeScaleMailbox,&init_time_scale);
    xQueueOverwrite(ScreenSelectorMailbox,&init_screen);

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
    if (xTaskCreate(NumberProcessTemp_thread, "NumberProcessTemp_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
        PRINTF("NumberProcessTemp_thread creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(Alarm_thread, "Alarm_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
        PRINTF("Alarm_thread creation failed!.\r\n");
        while (1);
    }
    led1_off();
    led2_off();

    vTaskStartScheduler();
    for (;;);
}

static void Alarm_thread(void *pvParameters){
    uint16_t alarm_value;
    uint8_t set_alarm;
    static uint16_t count_ss = 0;
    static uint16_t count_up_limit = 0;
    static uint16_t count_inferior_limit = 0;
    ////////////////////////////////////////////
    uint16_t alarmTemp_value;
    uint8_t set_alarmTemp;
    static uint16_t countTemp_ss = 0;
    static uint16_t count_upTemp_limit = 0;
    static uint16_t count_inferiorTemp_limit = 0;

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

        	if(count_up_limit > 50 || count_inferior_limit > 50 ){
        		set_alarm = 1;
        		xQueueSend(alarmQueue,&set_alarm,portMAX_DELAY);
        	}
        }
        ///////////////////////////////////////////////////////////////////////////////////
        if(xQueueReceive(NumberTempQueue,&alarmTemp_value ,pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
        	if (alarmTemp_value < inferiorTemp_limit){ //este viene de la queue CAMBIAR
          		count_inferiorTemp_limit++;
          		count_upTemp_limit = 0;
          		countTemp_ss = 0;
        	}
        	else if (alarmTemp_value > upTemp_limit){
        		count_upTemp_limit++;
        	    count_inferiorTemp_limit = 0;
        	    countTemp_ss = 0;
          	}
        	else{
        		count_inferiorTemp_limit = 0;
        		count_upTemp_limit = 0;
        		countTemp_ss ++;
        		if(countTemp_ss > 20){
					set_alarmTemp = 0;
					xQueueSend(alarmTempQueue,&set_alarmTemp,portMAX_DELAY);
        		}
        	}

        	if(count_upTemp_limit > 50 || count_inferiorTemp_limit > 50 ){
        		set_alarmTemp = 1;
        		xQueueSend(alarmTempQueue,&set_alarmTemp,portMAX_DELAY);
        	}
        }
        ////////////////////////////////////////////////////////////////
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
    point_str pointTemp_a = {0};
    point_str pointTemp_b = {0};
    uint16_t ypointGraphTemp;
    uint16_t ypointNumberTemp;
    uint8_t onedigitTemp = 0;
    uint8_t secdigitTemp = 0;
    uint8_t thrdigitTemp = 0;
    uint8_t screen_selected;
    static uint8_t set_alarm = 0;
    static uint8_t set_alarmTemp = 0;
    static uint8_t initial_clean1 = 1;
    static uint8_t initial_clean2 = 1;
    static uint8_t initial_clean3 = 1;

    for (;;){
    	if(xQueuePeek(ScreenSelectorMailbox,&screen_selected,pdMS_TO_TICKS(5))){
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
				}
			}

			if(xQueueReceive(alarmQueue,&set_alarm, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
				if (set_alarm && screen_selected ==1){
					LCD_nokia_write_string_xy_FB(1,2,"ALARM");
					led1_on();
				}
				else{
					LCD_nokia_write_string_xy_FB(1,2,"     ");
					led1_off();
				}
			}
			if(xQueueReceive(alarmTempQueue,&set_alarmTemp, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
				if (set_alarmTemp && screen_selected ==2){
					if(screen_selected==2){
						LCD_nokia_write_string_xy_FB(1,2,"ALARM");
					}
					led2_on();
				}
				else{
					LCD_nokia_write_string_xy_FB(1,2,"     ");
					led2_off();
				}
			}

		if(xQueueReceive(PointTempQueue,&ypointGraphTemp, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
			if(xQueuePeek(TimeScaleMailbox,&x_increment,pdMS_TO_TICKS(5))){
				pointTemp_a = pointTemp_b;
				pointTemp_b.y = ypointGraphTemp;
				if(pointTemp_b.x<84){
					pointTemp_b.x += x_increment;
				}
				else{
					pointTemp_b.x = 0;
					LCD_nokia_clear_range_FrameBuffer(0,3,252);
				}
			}
		}
    		if (screen_selected == 1){
				if(initial_clean1){
					LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
					initial_clean1 = 0;
				}
				drawline(point_a.x,point_a.y,point_b.x,point_b.y,50);
				if(xQueueReceive(NumberQueue,&ypointNumber, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
					LCD_nokia_clear_range_FrameBuffer(0,1,20);
					onedigit = ((ypointNumber/100)%10)+0x30;
					secdigit = ((ypointNumber/10)%10)+0x30;
					thrdigit = (ypointNumber%10)+0x30;
					LCD_nokia_write_char_xy_FB(0,1,onedigit);
					LCD_nokia_write_char_xy_FB(5,1,'.');
					LCD_nokia_write_char_xy_FB(10,1,secdigit);
					LCD_nokia_write_char_xy_FB(15,1,thrdigit);
					LCD_nokia_write_char_xy_FB(20,1,' ');
					LCD_nokia_write_string_xy_FB(25,1,"mv");
					taskYIELD();
				}
				initial_clean2 = 1;
    		}
        	if (screen_selected == 2 ){
				if(initial_clean2){
					LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
					initial_clean2 = 0;
				}

				drawline(pointTemp_a.x,pointTemp_a.y,pointTemp_b.x,pointTemp_b.y,50);
				if(xQueueReceive(NumberTempQueue,&ypointNumberTemp, pdMS_TO_TICKS(5)) != errQUEUE_EMPTY){
					LCD_nokia_clear_range_FrameBuffer(0,1,20);
					onedigitTemp = ((ypointNumberTemp/100)%10)+0x30;
					secdigitTemp = ((ypointNumberTemp/10)%10)+0x30;
					thrdigitTemp = (ypointNumberTemp%10)+0x30;
					LCD_nokia_write_char_xy_FB(0,1,onedigitTemp);
					LCD_nokia_write_char_xy_FB(5,1,secdigitTemp);
					LCD_nokia_write_char_xy_FB(10,1,'.');
					LCD_nokia_write_char_xy_FB(15,1,thrdigitTemp);
					LCD_nokia_write_char_xy_FB(20,1,' ');
					LCD_nokia_write_string_xy_FB(25,1,"°");
					taskYIELD();
				}
				initial_clean3 = 1;
        	}
    		if (screen_selected == 3){
    			if(initial_clean3){
    				LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
					initial_clean3 = 0;
    			}
    			if(point_b.x >= 84 || pointTemp_b.x >= 84){
    		       	point_b.x = 0;
    		       	pointTemp_b.x = 0;
    		       	LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
    			}

				drawline(point_a.x,(point_a.y)+25 ,point_b.x,(point_b.y)+25 ,50);
				drawline(pointTemp_a.x,pointTemp_a.y,pointTemp_b.x,pointTemp_b.y,50);
				initial_clean1 = 1;
    		}
        }
    }
}
static void GraphProcess_thread(void *pvParameters){
    adcConv_str adcConvVal;
    adcConv_str adcConvVal2;
    uint16_t processedVal;
    uint16_t processedVal2;
    uint8_t screen_selected;
    for (;;){
    	if(xQueuePeek(ScreenSelectorMailbox,&screen_selected,pdMS_TO_TICKS(5))){
    		if (screen_selected == 1 || screen_selected == 3){
        /*Wait for the ADC conversion to be avaliable*/
				if(xQueueReceive(AdcConversionQueue,&adcConvVal,portMAX_DELAY) != errQUEUE_EMPTY){
					/*Process the raw adc value to be in range from 0 to 43*/
					processedVal = ((adcConvVal.data * 6)/1025);
					xQueueSend(PointQueue,&processedVal,portMAX_DELAY);
					taskYIELD();
				}
    		}  //screen 2 option ECG
    		if (screen_selected == 2 || screen_selected == 3){
    			if(xQueueReceive(AdcConversionTempQueue,&adcConvVal2,portMAX_DELAY) != errQUEUE_EMPTY){
					processedVal2 = ((adcConvVal2.data * 6)/1025);
					xQueueSend(PointTempQueue,&processedVal2,portMAX_DELAY);
					taskYIELD();
    			}
    		}
        }
    }
}
static void NumberProcess_thread(void *pvParameters){
    adcConv_str adcConvVal;
    uint16_t processedVal;
    for (;;){
        /*Wait for the ADC conversion to be avaliable*/
        if(xQueueReceive(AdcConversionQueue,&adcConvVal,portMAX_DELAY) != errQUEUE_EMPTY){
            /*Process the raw adc value to be in range from 0 to 300 (3.0023mv)*/
            processedVal = ((adcConvVal.data * 7.33)/100);
            xQueueSend(NumberQueue,&processedVal,portMAX_DELAY);
            //send twice to read again
            xQueueSend(NumberQueue,&processedVal,portMAX_DELAY);
            taskYIELD();
        }
    }
}
static void NumberProcessTemp_thread(void *pvParameters){
    adcConv_str adcConvVal;
    uint16_t processedVal;
    for (;;){
        /*Wait for the ADC conversion to be avaliable*/
        if(xQueueReceive(AdcConversionTempQueue,&adcConvVal,portMAX_DELAY) != errQUEUE_EMPTY){
            /*Process the raw adc value to be in range from 340 to 400 (34 - 40)*/
            processedVal = 340+((adcConvVal.data * 60)/4096);
            xQueueSend(NumberTempQueue,&processedVal,portMAX_DELAY);
            //send twice to read again
            xQueueSend(NumberTempQueue,&processedVal,portMAX_DELAY);
            taskYIELD();
        }
    }
}
