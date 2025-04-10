/*
 * terminal.c
 *
 *  Created on: 8 abr. 2025
 *      Author: Jorge Leautaud
 */

#include "MK66F18.h"
#include "fsl_uart.h"
#include "fsl_lpuart.h"
#include "terminal.h"




void UART_SendString(const char *str) {
    LPUART_WriteBlocking(UART0, (const uint8_t *)str, strlen(str));
}



void UART_ReceiveString(char *buffer, uint32_t length) {
    uint32_t i = 0;
    char c;

    while(i < length - 1) {
        UART_ReadBlocking(UART0, (uint8_t *)&c, 1);

        if(c == '\r' || c == '\n') {
            buffer[i] = '\0';
            break;
        }

        buffer[i++] = c;
    }
}


void ShowConfigMenu(void) {
    UART_SendString("\r\n=== Configuracion de Alarmas ===\r\n");
    UART_SendString("1. Configurar pulsos cardiacos\r\n");
    UART_SendString("2. Configurar temperatura\r\n");
    UART_SendString("3. Mostrar configuracion actual\r\n");
    UART_SendString("4. Salir\r\n");
    UART_SendString("Seleccione una opcion: ");
}


void ConfigureHeartRateAlarms(void) {
    char buffer[BUFFER_SIZE];
    float high, low;

    UART_SendString("\r\n--- Configurar Pulsos Cardiacos ---\r\n");

    UART_SendString("Ingrese limite alto (ej. 2.85): ");
    UART_ReceiveString(buffer, BUFFER_SIZE);
    sscanf(buffer, "%f", &high);

    UART_SendString("Ingrese limite bajo (ej. 0.15): ");
    UART_ReceiveString(buffer, BUFFER_SIZE);
    sscanf(buffer, "%f", &low);

    current_settings.heart_rate_high = high;
    current_settings.heart_rate_low = low;

    UART_SendString("Valores actualizados.\r\n");
}


void ConfigureTempAlarms(void) {
    char buffer[BUFFER_SIZE];
    float high, low;

    UART_SendString("\r\n--- Configurar Temperatura ---\r\n");

    UART_SendString("Ingrese limite alto (ej. 37.5): ");
    UART_ReceiveString(buffer, BUFFER_SIZE);
    sscanf(buffer, "%f", &high);

    UART_SendString("Ingrese limite bajo (ej. 35.0): ");
    UART_ReceiveString(buffer, BUFFER_SIZE);
    sscanf(buffer, "%f", &low);

    current_settings.temp_high = high;
    current_settings.temp_low = low;

    UART_SendString("Valores actualizados.\r\n");
}

void ShowCurrentSettings(void) {
    char buffer[BUFFER_SIZE];

    snprintf(buffer, BUFFER_SIZE,
             "\r\n--- Configuracion Actual ---\r\n"
             "Pulsos cardiacos: Alto=%.2fmV, Bajo=%.2fmV\r\n"
             "Temperatura: Alto=%.1f°C, Bajo=%.1f°C\r\n",
             current_settings.heart_rate_high, current_settings.heart_rate_low,
             current_settings.temp_high, current_settings.temp_low);

    UART_SendString(buffer);
}

void UART_ConfigTask(void *pvParameters) {
    char option[BUFFER_SIZE];

    UART_SendString("\r\nSistema de Monitoreo Medico - UART Config\r\n");

    while(1) {
        ShowConfigMenu();
        UART_ReceiveString(option, BUFFER_SIZE);

        switch(option[0]) {
            case '1':
                ConfigureHeartRateAlarms();
                break;
            case '2':
                ConfigureTempAlarms();
                break;
            case '3':
                ShowCurrentSettings();
                break;
            case '4':
                UART_SendString("Saliendo del menu...\r\n");
                break;
            default:
                UART_SendString("Opcion no valida\r\n");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Pequeña pausa
    }
}







