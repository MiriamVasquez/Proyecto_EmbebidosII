/*
 * terminal.h
 *
 *  Created on: 8 abr. 2025
 *      Author: Jorge Leautaud
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "MK66F18.h"
#include "fsl_uart.h"
#include "fsl_lpuart.h"

typedef struct {
    float heart_rate_high;
    float heart_rate_low;
    float temp_high;
    float temp_low;
} AlarmSettings;


AlarmSettings current_settings = {
    .heart_rate_high = 2.85f,
    .heart_rate_low = 0.15f,
    .temp_high = 37.5f,
    .temp_low = 35.0f
};

#define BUFFER_SIZE 128




#endif /* TERMINAL_H_ */
