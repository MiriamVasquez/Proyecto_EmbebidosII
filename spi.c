/*
 *
 *Gabriela And Jorge
 *
 */

#include "spi.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "clock_config.h"

void init_SPI(void)
{
	dspi_master_config_t masterConfig;
	uint32_t srcClock_Hz;

	CLOCK_EnableClock(kCLOCK_Spi0);
	CLOCK_EnableClock(kCLOCK_PortD);	//habilita el reloj en los pines D

	PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAlt2);		//los pines de los cuales disponemos para usar el SPI
	PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt2);

/* Master config */
	masterConfig.whichCtar 							= kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate 				= TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.bitsPerFrame 			= FRAME_SIZE;
	masterConfig.ctarConfig.cpol					= kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha					= kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction				= kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec 	= TRANSFER_DELAY / TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = TRANSFER_DELAY / TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = TRANSFER_DELAY / TRANSFER_BAUDRATE;

	masterConfig.whichPcs = kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	masterConfig.enableContinuousSCK = false;
	masterConfig.enableRxFifoOverWrite = false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint = kDSPI_SckToSin0Clock;

	srcClock_Hz = CLOCK_GetFreq(DSPI0_CLK_SRC);
	DSPI_MasterInit(SPI0, &masterConfig, srcClock_Hz);

}
