/*
 * pit.h
 *
 *  Created on: Sep 20, 2024
 *      Author: Miriam
 */

#ifndef PIT_H_
#define PIT_H_

#include "fsl_pit.h"
#include "stdint.h"

#define ONESECOND 	(1000000U)
#define ONEHOUR 	(3600000000)
#define HALFMINUTE 	(30000000)
#define threeMINUTE (90030000)
#define TIMECHECK 	(60001000)
#define FRAME_REFRESH_TIME (500000U)
#define FS 			(63U)

#define PIT_IRQ_ID        PIT0_IRQn
#define PIT_IRQ_ID_1      PIT1_IRQn
#define PIT_IRQ_ID_2      PIT2_IRQn

void start_pit (uint32_t time_us,pit_chnl_t pit_channel);
void stop_pit (pit_chnl_t pit_channel);
void pit_config(void);
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PIT2_IRQHandler(void);
void PIT3_IRQHandler(void);
void pit_callback_init(pit_chnl_t pit_channel, void (*handler)(void));



#endif /* PIT_H_ */
