/*
 * spi.h
 *
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-12
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

extern volatile uint8_t spi_buf;

#define CS          BIT0
#define RESET       BIT4
#define SCLK        BIT5
#define MOSI        BIT0
#define MISO        BIT1
#define DIO_0       BIT3

/*
 * Initialize the hardware
 */
void spi_init(void);
void spi_txready();
void spi_rxready();
void spi_send(uint8_t data);
void spi_recv();
void spi_transfer(uint8_t data);
void spi_chipEnable();
void spi_chipDisable();
void dio0irq_init();

#endif /* SPI_H_ */
