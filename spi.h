/*
 *
 *Gabriela And Jorge
 *
 */

#ifndef SPI_H_
#define SPI_H_

#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */

#define TRANSFER_SIZE     64U     /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 1000000U /*! Transfer baudrate - 500k */
#define TRANSFER_DELAY 1000000000U
#define FRAME_SIZE 8u

/*
 * Inicializamos el SPI con sus características de maestro exclavo
 */
void init_SPI(void);

#endif
