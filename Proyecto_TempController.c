/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Proyecto_TempController.c
 * @brief   Application entry point.
 */

//sensor
/*
 * azul -> SDA
 * cafe -> SCL
 * negro -> gnd
 * blanco -> vin
 */



/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_ftm.h"
#include "fsl_i2c.h"

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
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define GrapNumb_PRIORITY   (configMAX_PRIORITIES - 2)
#define X_INCREMENT 1


// Definiciones para el PWM
#define FTM_PWM_BASEADDR FTM2
#define FTM_PWM_CHANNEL kFTM_Chnl_0
#define FTM_PWM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define PWM_FREQUENCY 25000 // 25 kHz para ventilador
#define PWM_DUTY_CYCLE_MIN 20 // Mínimo duty cycle (20%)
#define PWM_DUTY_CYCLE_MAX 100 // Máximo duty cycle (100%)

//sensor cmnds
#define SHT31_ADDR 0x44
#define SHT31_SOFT_RESET 0x30A2
#define SHT31_MEASUREMENT_HIGHREP 0x2C06

typedef struct{
	uint8_t x;
	uint8_t y;
}point_str;



typedef struct{
	uint16_t temp_up;
	uint16_t temp_low;
	uint16_t Hum_up;
	uint16_t Hum_low;
}limits_str;

TimerHandle_t SendFBTimer;

QueueHandle_t Sht31DataQueue;
QueueHandle_t TimeScaleMailbox;
QueueHandle_t ScreenSelectorMailbox;

QueueHandle_t PointQueue;
QueueHandle_t PointTempQueue;
QueueHandle_t NumberQueue;
QueueHandle_t NumberTempQueue;
QueueHandle_t alarmQueue;
QueueHandle_t alarmTempQueue;
QueueHandle_t limitsQueue;
QueueHandle_t UartRxQueue;
SemaphoreHandle_t xUartMutex;
SemaphoreHandle_t xLCDMutex;




/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void LCDprint_thread(void *pvParameters);
static void Alarm_thread(void *pvParameters);
static void Terminal_thread(void *pvParameters);
static void SHT31_ReadTask(void *pvparameters);
static void PWM_Init(void);
static void PWM_SetDutyCycle(uint8_t dutyCycle);
static void FanControl_task(void *pvParameters);


uint8_t CRC(const uint8_t *data, uint8_t len);

/* Task priorities. */
bool SHT31_WriteCommand(I2C_Type *i2cBase, uint16_t command) {
    uint8_t cmd[2] = {command >> 8, command & 0xFF};

    i2c_master_transfer_t transfer;
    transfer.slaveAddress = SHT31_ADDR;
    transfer.direction = kI2C_Write;
    transfer.subaddress = 0;
    transfer.subaddressSize = 0;
    transfer.data = cmd;
    transfer.dataSize = 2;
    transfer.flags = kI2C_TransferDefaultFlag;

    if (I2C_MasterTransferBlocking(i2cBase, &transfer) != kStatus_Success) {
        return false;
    }

    // Esperar 1ms entre comandos (requerido por el datasheet)
    SDK_DelayAtLeastUs(1000, SystemCoreClock);
    return true;
}

// Función para leer datos del sensor
bool SHT31_ReadData(I2C_Type *i2cBase, uint16_t *temp, uint16_t *humidity) {
    uint8_t data[6];

    i2c_master_transfer_t transfer;
    transfer.slaveAddress = SHT31_ADDR;
    transfer.direction = kI2C_Read;
    transfer.subaddress = 0;
    transfer.subaddressSize = 0;
    transfer.data = data;
    transfer.dataSize = 6;
    transfer.flags = kI2C_TransferDefaultFlag;

    if (I2C_MasterTransferBlocking(i2cBase, &transfer) != kStatus_Success) {
        return false;
    }

    // Verificar CRC (opcional pero recomendado)
    if (data[2] != CRC(data, 2) || data[5] != CRC(data+3, 2)) {
        return false;
    }

    *temp = (data[0] << 8) | data[1];
    *humidity = (data[3] << 8) | data[4];
    return true;
}

// Función de inicialización del sensor
bool SHT31_Init(I2C_Type *i2cBase) {
    // 1. Reset suave del sensor
    if (!SHT31_WriteCommand(i2cBase, SHT31_SOFT_RESET)) {
        return false;
    }

    // Esperar 1.5ms después del reset (máximo según datasheet)
    SDK_DelayAtLeastUs(1500, SystemCoreClock);

    return true;
}

// Función para leer temperatura y humedad
bool SHT31_ReadTempHum(I2C_Type *i2cBase, float *temp, float *humidity) {
    uint16_t rawTemp, rawHum;

    // Iniciar medición en modo alta precisión con clock stretching
    if (!SHT31_WriteCommand(i2cBase, SHT31_MEASUREMENT_HIGHREP)) {
        return false;
    }

    // Esperar tiempo de medición (máximo 15ms para alta precisión)
    SDK_DelayAtLeastUs(15000, SystemCoreClock);

    // Leer datos
    if (!SHT31_ReadData(i2cBase, &rawTemp, &rawHum)) {
        return false;
    }

    // Convertir valores crudos a unidades físicas
    *temp = -45.0f + 175.0f * (rawTemp / 65535.0f);
    *humidity = 100.0f * (rawHum / 65535.0f);

    return true;
}

// Función CRC8 (requerida para verificación de datos)
uint8_t CRC(const uint8_t *data, uint8_t len) {
    const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;

    for (uint8_t j = len; j > 0; j--) {
        crc ^= *data++;

        for (uint8_t i = 8; i > 0; i--) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
        }
    }
    return crc;
}

void ScreenInit(){
	init_SPI();
	LCD_nokia_init(); /*! Configuration function for the LCD */
	LCD_nokia_clear();
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


void screen_selector(uint32_t flags)
{
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


void UART0_RX_TX_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    char c;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART0)) {
        c = UART_ReadByte(UART0);  // Leer el byte recibido

        // Enviar a la cola desde ISR
        xQueueSendFromISR(UartRxQueue, &c, &xHigherPriorityTaskWoken);

        // Forzar cambio de contexto si es necesario
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int main(void) {


	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_I2c0);
	CLOCK_SetSimSafeDivs();
	uint8_t init_time_scale = 1;
	uint8_t init_screen = 2;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    EnableIRQ(UART0_RX_TX_IRQn);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);

	// Crear cola para caracteres UART
	UartRxQueue = xQueueCreate(32, sizeof(char));  // Buffer para 32 caracteres
	xUartMutex = xSemaphoreCreateMutex();

	// Habilitar interrupción por recepción en la UART
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable);

    ScreenInit();
    PWM_Init();

    init_button(sw3);
    init_button(sw2);
    gpio_init_as_output();
    gpio_init_i2c();

    NVIC_set_basepri_threshold(PRIORITY_10);
	NVIC_enable_interrupt_and_priotity(PIT_CH1_IRQ,PRIORITY_4);
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_4);
	NVIC_enable_interrupt_and_priotity(PORTD_IRQ,PRIORITY_4);
	NVIC_enable_interrupt_and_priotity(PIT1_IRQn, PRIORITY_1);
	NVIC_global_enable_interrupts;

	set_button_as_interrupt(sw3);
	set_button_as_interrupt(sw2);
    // Inicializar I2C
    i2c_master_config_t config;
    I2C_MasterGetDefaultConfig(&config);
    config.baudRate_Bps = 100000; // 100kHz inicialmente
    I2C_MasterInit(I2C0, &config, CLOCK_GetFreq(kCLOCK_BusClk));



    if (!SHT31_Init(I2C0)) {
        PRINTF("Error al inicializar SHT31\r\n");
        while(1);
    }




	GPIO_callback_init(GPIO_A, time_scale);
	GPIO_callback_init(GPIO_D, screen_selector);


	Sht31DataQueue = xQueueCreate(5, sizeof(float));
	PointQueue = xQueueCreate(5,sizeof(uint16_t));
	PointTempQueue = xQueueCreate(5,sizeof(uint16_t));
	NumberQueue = xQueueCreate(5,sizeof(uint16_t));
	NumberTempQueue = xQueueCreate(5,sizeof(uint16_t));
	alarmQueue = xQueueCreate(1,sizeof(uint8_t));
	alarmTempQueue = xQueueCreate(1,sizeof(uint8_t));
	limitsQueue = xQueueCreate(4,sizeof(limits_str));

	TimeScaleMailbox = xQueueCreate(1,sizeof(uint8_t));
	ScreenSelectorMailbox = xQueueCreate(1,sizeof(uint8_t));

	SendFBTimer = xTimerCreate(
			"WriteFB", /*Timer's Name*/
			pdMS_TO_TICKS(33), /* Expiration time = 33ms*/
			pdTRUE, /*Auto-reload mode*/
			0, /*Timer's ID. Not used*/
			SendFBCallback); /*Auto-Reload notification function. Called at expiration time*/
	/*Start FrameBuffer update timer*/
	xTimerStart(SendFBTimer,0);

	/*Publish time-scale init value*/
	xQueueOverwrite(TimeScaleMailbox,&init_time_scale);
	xQueueOverwrite(ScreenSelectorMailbox,&init_screen);

	xLCDMutex = xSemaphoreCreateMutex();

	if (xLCDMutex == NULL) {
	    PRINTF("Failed to create LCD mutex.\r\n");
	    while(1);
	}

	if (xTaskCreate(LCDprint_thread, "LCDprint_thread", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY, NULL) !=pdPASS){
		PRINTF("LCDprint_thread creation failed!.\r\n");
		while (1);
	}

	if (xTaskCreate(Alarm_thread, "Alarm_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
		PRINTF("Alarm_thread creation failed!.\r\n");
		while (1);
	}

	if (xTaskCreate(Terminal_thread, "Terminal_thread", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) !=pdPASS){
		PRINTF("terminal_thread creation failed!.\r\n");
		while (1);
	}

	if (xTaskCreate(FanControl_task, "FanCtrl", configMINIMAL_STACK_SIZE + 100, NULL, GrapNumb_PRIORITY, NULL) != pdPASS) {
	    PRINTF("FanControl_task creation failed!.\r\n");
	    while (1);
	}

    if (xTaskCreate(SHT31_ReadTask, "SHT31_Read", configMINIMAL_STACK_SIZE + 200, NULL, GrapNumb_PRIORITY, NULL) != pdPASS) {
        PRINTF("SHT31_ReadTask creation failed!.\r\n");
        while (1);
    }


	led1_off();
	led2_off();

	vTaskStartScheduler();
	for(;;);
}




static void Alarm_thread(void *pvParameters) {
    uint16_t hum_value;
    uint16_t temp_value;
    uint8_t set_alarm;
    uint8_t set_alarmTemp;
    static uint16_t count_ss = 0;
    static uint16_t count_up_limit = 0;
    static uint16_t count_inferior_limit = 0;
    static uint16_t countTemp_ss = 0;
    static uint16_t count_upTemp_limit = 0;
    static uint16_t count_inferiorTemp_limit = 0;
    limits_str current_limits;

    // Valores por defecto
    current_limits.Hum_up = 600;   // 60.0% humedad (*10)
    current_limits.Hum_low = 300;  // 30.0% humedad (*10)
    current_limits.temp_up = 400;  // 40.0°C (*10)
    current_limits.temp_low = 200; // 20.0°C (*10)
    xQueueSend(limitsQueue, &current_limits, portMAX_DELAY);

    for (;;) {
        // Verificar si hay nuevos límites
        if (xQueueReceive(limitsQueue, &current_limits, 0) == pdPASS);

        // Procesar humedad
        if (xQueueReceive(NumberQueue, &hum_value, pdMS_TO_TICKS(5)) == pdPASS) {
            if (hum_value < current_limits.Hum_low) {
                count_inferior_limit++;
                count_up_limit = 0;
                count_ss = 0;
            }
            else if (hum_value > current_limits.Hum_up) {
                count_up_limit++;
                count_inferior_limit = 0;
                count_ss = 0;
            }
            else {
                count_inferior_limit = 0;
                count_up_limit = 0;
                count_ss++;
                if (count_ss > 20) {
                    set_alarm = 0;
                    xQueueOverwrite(alarmQueue, &set_alarm);
                }
            }

            if (count_up_limit > 10 || count_inferior_limit > 10) {
                set_alarm = 1;
                xQueueOverwrite(alarmQueue, &set_alarm);
            }
        }

        // Procesar temperatura
        if (xQueueReceive(NumberTempQueue, &temp_value, pdMS_TO_TICKS(5)) == pdPASS) {
            if (temp_value < current_limits.temp_low) {
                count_inferiorTemp_limit++;
                count_upTemp_limit = 0;
                countTemp_ss = 0;
            }
            else if (temp_value > current_limits.temp_up) {
                count_upTemp_limit++;
                count_inferiorTemp_limit = 0;
                countTemp_ss = 0;
            }
            else {
                count_inferiorTemp_limit = 0;
                count_upTemp_limit = 0;
                countTemp_ss++;
                if (countTemp_ss > 20) {
                    set_alarmTemp = 0;
                    xQueueOverwrite(alarmTempQueue, &set_alarmTemp);
                }
            }

            if (count_upTemp_limit > 10 || count_inferiorTemp_limit > 10) {
                set_alarmTemp = 1;
                xQueueOverwrite(alarmTempQueue, &set_alarmTemp);
            }
        }
    }
}

static void LCDprint_thread(void *pvParameters) {
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

    uint8_t fan_speed_display = 0;
    char fan_status_str[10] = "OFF";

    for (;;) {
    	if (xSemaphoreTake(xLCDMutex, portMAX_DELAY) == pdTRUE) {
    	            if(xQueuePeek(ScreenSelectorMailbox,&screen_selected,pdMS_TO_TICKS(5))) {
    	                // Pantalla 1: Humedad
    	                if (screen_selected == 1) {
    	                    if(initial_clean1) {
    	                        LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
    	                        LCD_nokia_write_string_xy_FB(0,0,"Humedad:");
    	                        initial_clean1 = 0;
    	                    }

    	                    // Mostrar valor numérico
    	                    if(xQueueReceive(NumberQueue,&ypointNumber, pdMS_TO_TICKS(5))) {
    	                        LCD_nokia_clear_range_FrameBuffer(0,1,30);
    	                        onedigit = ((ypointNumber/100)%10)+0x30; // Entero
    	                        secdigit = ((ypointNumber/10)%10)+0x30;   // Decimal
    	                        LCD_nokia_write_char_xy_FB(0,1,onedigit);
    	                        LCD_nokia_write_char_xy_FB(5,1,'.');
    	                        LCD_nokia_write_char_xy_FB(10,1,secdigit);
    	                        LCD_nokia_write_char_xy_FB(15,1,'%');
    	                    }

    	                    // Mostrar gráfico
    	                    drawline(point_a.x,point_a.y,point_b.x,point_b.y,50);
    	                }

    	                // Pantalla 2: Temperatura
    	                else if (screen_selected == 2) {
    	                    if(initial_clean2) {
    	                        LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
    	                        LCD_nokia_write_string_xy_FB(0,0,"Temperatura:");
    	                        initial_clean2 = 0;
    	                    }

    	                    // Mostrar valor numérico
    	                    if(xQueueReceive(NumberTempQueue,&ypointNumberTemp, pdMS_TO_TICKS(5))) {
    	                        LCD_nokia_clear_range_FrameBuffer(0,1,30);
    	                        onedigitTemp = ((ypointNumberTemp/100)%10)+0x30; // Entero
    	                        secdigitTemp = ((ypointNumberTemp/10)%10)+0x30;  // Decimal
    	                        LCD_nokia_write_char_xy_FB(0,1,onedigitTemp);
    	                        LCD_nokia_write_char_xy_FB(5,1,secdigitTemp);
    	                        LCD_nokia_write_char_xy_FB(10,1,'.');
    	                        LCD_nokia_write_char_xy_FB(15,1,'C');
    	                    }

    	                    // Mostrar gráfico
    	                    drawline(pointTemp_a.x,pointTemp_a.y,pointTemp_b.x,pointTemp_b.y,50);
    	                }

    	                // Pantalla 3: Ambas mediciones
    	                else if (screen_selected == 3) {
    	                    if(initial_clean3) {
    	                        LCD_nokia_clear_range_FrameBuffer(0, 0, 504);
    	                        LCD_nokia_write_string_xy_FB(0,0,"Temp:");
    	                        LCD_nokia_write_string_xy_FB(0,2,"Hum:");
    	                        initial_clean3 = 0;
    	                    }

    	                    // Mostrar ambos gráficos
    	                    drawline(point_a.x,(point_a.y)+25,point_b.x,(point_b.y)+25,50);
    	                    drawline(pointTemp_a.x,pointTemp_a.y,pointTemp_b.x,pointTemp_b.y,50);
    	                }
    	            }
    	            xSemaphoreGive(xLCDMutex);
    	        }
    	        vTaskDelay(pdMS_TO_TICKS(1));
    }
}




static void Terminal_thread(void *pvParameters) {
    limits_str current_limits;
    char input_char;
    uint32_t input_value = 0;
    uint8_t menu_state = 0; // 0:menu principal, 1:ECG alto, 2:ECG bajo, 3:Temp alta, 4:Temp baja
    uint8_t decimal_count = 0;
    uint8_t decimal_flag = 0;

    // Valores por defecto actualizados para humedad y temperatura
    current_limits.Hum_up = 600;   // 60.0% humedad (*10)
    current_limits.Hum_low = 300;  // 30.0% humedad (*10)
    current_limits.temp_up = 400;  // 40.0°C (*10)
    current_limits.temp_low = 200; // 20.0°C (*10)
    xQueueSend(limitsQueue, &current_limits, portMAX_DELAY);

    // Mostrar menú inicial con unidades correctas
    PRINTF("\r\n=== Configuracion de Alarmas ===");
    PRINTF("\r\n1. Limite alto Humedad (%.1f %%)", current_limits.Hum_up/10.0f);
    PRINTF("\r\n2. Limite bajo Humedad (%.1f %%)", current_limits.Hum_low/10.0f);
    PRINTF("\r\n3. Limite alto Temperatura (%.1f C)", current_limits.temp_up/10.0f);
    PRINTF("\r\n4. Limite bajo Temperatura (%.1f C)", current_limits.temp_low/10.0f);
    PRINTF("\r\nSeleccione opcion (1-4): ");

    for (;;) {
        // Esperar bloqueado hasta que llegue un carácter
        if (xQueueReceive(UartRxQueue, &input_char, portMAX_DELAY) == pdPASS) {
            // Solo procesar caracteres válidos
            if ((input_char >= '0' && input_char <= '9') || input_char == '.' ||
                input_char == '\r' || input_char == '\n' || input_char == 8 || input_char == 127) {

                // Proteger acceso a la UART con mutex
                if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) {
                    PUTCHAR(input_char); // Echo del carácter
                    xSemaphoreGive(xUartMutex);
                }

                if (input_char == '\r' || input_char == '\n') {
                    // Procesar entrada completa
                    if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) {
                        if (menu_state == 0) {
                            // Menú principal
                            if (input_value >= 1 && input_value <= 5) {
                                switch(input_value) {
                                case 1:
                                    PRINTF("\r\nIngrese nuevo limite alto Humedad (mV): ");
                                    menu_state = 1;
                                    break;
                                case 2:
                                    PRINTF("\r\nIngrese nuevo limite bajo Humedad (mV): ");
                                    menu_state = 2;
                                    break;
                                case 3:
                                    PRINTF("\r\nIngrese nuevo limite alto temperatura (C): ");
                                    menu_state = 3;
                                    break;
                                case 4:
                                    PRINTF("\r\nIngrese nuevo limite bajo temperatura (C): ");
                                    menu_state = 4;
                                    break;
                                }
                            } else {
                                PRINTF("\r\nOpcion no valida.");
                            }
                        } else {
                            // Modo de entrada de valores
                            float float_value = (float)input_value;

                            // Ajustar por decimales
                            if (decimal_flag) {
                                for (uint8_t i = 0; i < decimal_count; i++) {
                                    float_value /= 10.0f;
                                }
                            }

                            switch(menu_state) {
                            case 1:
                                current_limits.Hum_up = (uint16_t)(float_value * 100);
                                PRINTF("\r\nNuevo limite alto Humedad: %.2f mV", float_value);
                                break;
                            case 2:
                                current_limits.Hum_low = (uint16_t)(float_value * 100);
                                PRINTF("\r\nNuevo limite bajo Humedad: %.2f mV", float_value);
                                break;
                            case 3:
                                current_limits.temp_up = (uint16_t)(float_value * 10);
                                PRINTF("\r\nNuevo limite alto temperatura: %.1f C", float_value);
                                break;
                            case 4:
                                current_limits.temp_low = (uint16_t)(float_value * 10);
                                PRINTF("\r\nNuevo limite bajo temperatura: %.1f C", float_value);
                                break;
                            }

                            xQueueSend(limitsQueue, &current_limits, portMAX_DELAY);
                            menu_state = 0;
                        }

                        // Mostrar menú principal
                        if (menu_state == 0) {
                            PRINTF("\r\n\r\n1. Limite alto Hum (%.2f mV)", current_limits.Hum_up/100.0f);
                            PRINTF("\r\n2. Limite bajo Hum (%.2f mV)", current_limits.Hum_low/100.0f);
                            PRINTF("\r\n3. Limite alto temperatura (%.1f C)", current_limits.temp_up/10.0f);
                            PRINTF("\r\n4. Limite bajo temperatura (%.1f C)", current_limits.temp_low/10.0f);
                            PRINTF("\r\nSeleccione opcion (1-4): ");
                        }
                        xSemaphoreGive(xUartMutex);
                    }

                    // Resetear valores
                    input_value = 0;
                    decimal_count = 0;
                    decimal_flag = 0;
                }
                else if (input_char == 8 || input_char == 127) { // Backspace/Delete
                    if (xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE) {
                        PUTCHAR(' ');
                        PUTCHAR('\b');
                        xSemaphoreGive(xUartMutex);
                    }
                    input_value /= 10;
                }
                else if (input_char == '.') {
                    decimal_flag = 1;
                }
                else if (input_char >= '0' && input_char <= '9') {
                    input_value = input_value * 10 + (input_char - '0');
                    if (decimal_flag) {
                        decimal_count++;
                    }
                }
            }
        }
    }
}

static void PWM_Init(void)
{
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t pwmParam;

    // Configurar el reloj para FTM0
    CLOCK_EnableClock(kCLOCK_Ftm2);
    CLOCK_SetTpmClock(1); // Seleccionar reloj del bus

    // Configuración básica del FTM
    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_16;
    FTM_Init(FTM_PWM_BASEADDR, &ftmInfo);

    // Configurar el pin para PWM
    PORT_SetPinMux(PORTB, 18, kPORT_MuxAlt3); // FTM0 CH3 en PTD4

    // Configurar parámetros PWM
    pwmParam.chnlNumber = FTM_PWM_CHANNEL;
    pwmParam.level = kFTM_HighTrue;
    pwmParam.dutyCyclePercent = 0; // Iniciar apagado
    pwmParam.firstEdgeDelayPercent = 0;

    // Configurar PWM
    FTM_SetupPwm(FTM_PWM_BASEADDR, &pwmParam, 1, kFTM_EdgeAlignedPwm, PWM_FREQUENCY, FTM_PWM_SOURCE_CLOCK);

    // Iniciar el timer
    FTM_StartTimer(FTM_PWM_BASEADDR, kFTM_SystemClock);
}

static void PWM_SetDutyCycle(uint8_t dutyCycle)
{
    // Limitar el duty cycle entre mínimo y máximo
    if (dutyCycle < PWM_DUTY_CYCLE_MIN) dutyCycle = 0; // Apagar si es menor al mínimo
    else if (dutyCycle > PWM_DUTY_CYCLE_MAX) dutyCycle = PWM_DUTY_CYCLE_MAX;

    // Actualizar el duty cycle
    FTM_UpdatePwmDutycycle(FTM_PWM_BASEADDR, FTM_PWM_CHANNEL, kFTM_EdgeAlignedPwm, dutyCycle);
}

static void FanControl_task(void *pvParameters) {
    float current_temp = 0.0f;
    limits_str current_limits;
    uint8_t fan_state = 0;
    uint8_t fan_speed = 0;

    // Valores por defecto
    current_limits.temp_up = 400;  // 40.0°C
    current_limits.temp_low = 200; // 20.0°C

    for (;;) {
        // Obtener temperatura actual
        if (xQueueReceive(Sht31DataQueue, &current_temp, pdMS_TO_TICKS(100))) {
            // Se recibió nueva temperatura
        }

        // Obtener los límites actuales
        if (xQueueReceive(limitsQueue, &current_limits, 0)) {
            // Se actualizaron los límites
        }

        // Control del ventilador
        if (current_temp > (current_limits.temp_up / 10.0f)) {
            // Temperatura alta - encender ventilador
            if (!fan_state) {
                fan_state = 1;
                fan_speed = PWM_DUTY_CYCLE_MAX;
                PWM_SetDutyCycle(fan_speed);
                PRINTF("Ventilador ENCENDIDO (Temp: %.1f°C > %.1f°C)\r\n",
                      current_temp, current_limits.temp_up / 10.0f);
            }

            // Control proporcional
            float temp_diff = current_temp - (current_limits.temp_up / 10.0f);
            float max_diff = 10.0f;
            if (temp_diff > max_diff) temp_diff = max_diff;

            fan_speed = PWM_DUTY_CYCLE_MIN +
                       (uint8_t)((PWM_DUTY_CYCLE_MAX - PWM_DUTY_CYCLE_MIN) * (temp_diff / max_diff));
            PWM_SetDutyCycle(fan_speed);
        }
        else if (current_temp < (current_limits.temp_low / 10.0f)) {
            // Temperatura baja - apagar ventilador
            if (fan_state) {
                fan_state = 0;
                fan_speed = 0;
                PWM_SetDutyCycle(0);
                PRINTF("Ventilador APAGADO (Temp: %.1f°C < %.1f°C)\r\n",
                      current_temp, current_limits.temp_low / 10.0f);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


static void SHT31_ReadTask(void *pvparameters) {
    float temp, hum;
    uint16_t temp_scaled, hum_scaled;

    for (;;) {
        if (SHT31_ReadTempHum(I2C0, &temp, &hum)) {
            // Escalar valores para facilitar manejo (temp*10 y hum*10 para mantener 1 decimal)
            temp_scaled = (uint16_t)(temp * 10);
            hum_scaled = (uint16_t)(hum * 10);

            // Enviar datos a las colas correspondientes
            xQueueSend(PointTempQueue, &temp_scaled, 0);
            xQueueSend(NumberTempQueue, &temp_scaled, 0);
            xQueueSend(PointQueue, &hum_scaled, 0);
            xQueueSend(NumberQueue, &hum_scaled, 0);

            // También enviar a FanControl (usaremos una nueva cola)
            xQueueSend(Sht31DataQueue, &temp, 0);
        } else {
            PRINTF("Error reading SHT31 sensor\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Leer cada segundo
    }
}
