/*
 * W25Qxx.h
 *
 *  Created on: May 12, 2024
 *      Author: Mahmudul Islam
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_
#include <stdint.h>


//void w25q_reset(void);
//uint32_t w25q_readID();

void W25Q_Reset (void);
uint32_t W25Q_ReadID (void);

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);


void W25Q_Reset_DMA (void);
uint32_t W25Q_ReadID_DMA (void);

#endif /* INC_W25QXX_H_ */
