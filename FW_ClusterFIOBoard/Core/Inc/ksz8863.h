/*
 * ksz8863.h
 *
 *  Created on: May 14, 2024
 *      Author: Asus
 */

#ifndef INC_KSZ8863_H_
#define INC_KSZ8863_H_
#include <stdint.h>



void ksz_csLOW (void);
void ksz_csHIGH (void);
void ksz_SPI_Write (uint8_t *data, uint8_t len);
void ksz_SPI_Read (uint8_t *data, uint32_t len);


uint32_t ksz_ReadID(void);
uint8_t ksz_Read(void);
uint8_t ksz_write(void);

uint32_t ksz_ReadID_DMA(void);
uint8_t ksz_Read_DMA(void);

#endif /* INC_KSZ8863_H_ */
