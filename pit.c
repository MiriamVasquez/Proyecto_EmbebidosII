/*
 \file
 	 This is the source file for the NVIC device driver for Kinetics k64
 	 It contains some configuration functions and runtime functions
 	 It is implemented using CMSIS Core functions
 \autor Miriam A. Vasquez Juarez, ie728332@iteso.mx
 \date 07/09/2024
 \todo
 	 Pending Interrupts and disable interrumpts are not enable
 */
#include "fsl_pit.h"
#include "pit.h"


#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define COUNT ((1) * (PIT_SOURCE_CLOCK) / 1000000)

static void (*pit_channel0_callback)(void) = 0;
static void (*pit_channel1_callback)(void) = 0;
static void (*pit_channel2_callback)(void) = 0;
static void (*pit_channel3_callback)(void) = 0;


void pit_callback_init(pit_chnl_t pit_channel, void (*handler)(void)){
	switch(pit_channel){
		case kPIT_Chnl_0:
			pit_channel0_callback = handler;
			break;
		case kPIT_Chnl_1:
			pit_channel1_callback = handler;
			break;
		case kPIT_Chnl_2:
			pit_channel2_callback = handler;
			break;
		case kPIT_Chnl_3:
			pit_channel3_callback = handler;
			break;
	}
}

void PIT0_IRQHandler(void){
	if(pit_channel0_callback){
		pit_channel0_callback();
		//Salta a la funcion que definimos como callback
	}
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    __DSB();
}

//void PIT1_IRQHandler(void){
//	if(pit_channel1_callback){
//		pit_channel1_callback();
//		//Salta a la funcion que definimos como callback
//	}
//    PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);
//    __DSB();
//}

void PIT2_IRQHandler(void){
	if(pit_channel2_callback){
		pit_channel2_callback();
		//Salta a la funcion que definimos como callback
	}
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_2, kPIT_TimerFlag);
    __DSB();
}

void PIT3_IRQHandler(void){
	if(pit_channel3_callback){
		pit_channel3_callback();
		//Salta a la funcion que definimos como callback
	}
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_3, kPIT_TimerFlag);
    __DSB();
}

void start_pit (uint32_t time_us,pit_chnl_t pit_channel){
    //PIT_SetTimerPeriod(PIT, pit_channel, USEC_TO_COUNT(time_us, PIT_SOURCE_CLOCK));
    PIT_SetTimerPeriod(PIT, pit_channel, COUNT*time_us);
    PIT_StartTimer(PIT, pit_channel);
    EnableIRQ(PIT_IRQ_ID);
    EnableIRQ(PIT_IRQ_ID_1);
    EnableIRQ(PIT_IRQ_ID_2);
}

void stop_pit (pit_chnl_t pit_channel){
	PIT_StopTimer(PIT, pit_channel);

}

void pit_config(void){
	pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);

    /* Enable timer interrupts for channels */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    PIT_EnableInterrupts(PIT, kPIT_Chnl_1, kPIT_TimerInterruptEnable);
    PIT_EnableInterrupts(PIT, kPIT_Chnl_2, kPIT_TimerInterruptEnable);
    PIT_EnableInterrupts(PIT, kPIT_Chnl_3, kPIT_TimerInterruptEnable);
}


