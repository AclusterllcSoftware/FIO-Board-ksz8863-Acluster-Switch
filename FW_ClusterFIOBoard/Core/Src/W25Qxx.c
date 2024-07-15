/*
 * W25Qxx.c
 *
 *  Created on: May 12, 2024
 *      Author: Mahmudul Islam
 */

#include "W25Qxx.h"
#include "main.h"

//extern SPI_HandleTypeDef hspi1;
//#define W25Q_SPI hspi1
//extern UART_HandleTypeDef huart3;
//
//
//#define csLOW() HAL_GPIO_WritePin (Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET)
//#define csHIGH() HAL_GPIO_WritePin (Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET)
//
//
//#define numBlock 32 // num of block for 16MB memory
//
//void w25q_reset(void)
//{
//	uint8_t tData[2];
//	tData[0] = 0x66;
//	tData[1] = 0x99;
//
//	csLOW();
//	HAL_SPI_Transmit(&W25Q_SPI, tData, 2, 2000);
//	csHIGH();
//	HAL_Delay(100);
//}
//
//
//uint32_t w25q_readID()
//{
//	uint8_t tData = 0x9F;
//	uint8_t rData[3];
//
//	csLOW();
//	HAL_SPI_Transmit(&W25Q_SPI, tData, 1, 2000);
//	HAL_SPI_Transmit(&W25Q_SPI, rData, 3, 2000);
//	csHIGH();
//
//	HAL_UART_Transmit(&huart3, rData, sizeof(rData), HAL_MAX_DELAY);
//	return (rData[0] << 16 | rData[1] << 8 | rData[2]);
//}





extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;



#define W25Q_SPI hspi1

#define numBLOCK 32  // number of total blocks for 16Mb flash, 32x16x16 pages and 32x16x16x256 Bytes

void W25Q_Delay(uint32_t time)
{
	HAL_Delay(time);
}

void csLOW (void)
{
	HAL_GPIO_WritePin (Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH (void)
{
	HAL_GPIO_WritePin (Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

/**************************************************************************************************/

void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	W25Q_Delay(100);
}

uint32_t W25Q_ReadID (void)
{
//	uint8_t tData = 0x9F;  // Read JEDEC ID
	uint8_t tData[1] = {0x9F};  // Read JEDEC ID
	uint8_t rData[3];
	csLOW();
//	SPI_Write(&tData, 1);
	SPI_Write(tData, 1);
	SPI_Read(rData, 3);
	csHIGH();
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
	}
	else
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 5);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}

void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock
	}
	else
	{
		tData[0] = 0x0C;  // Fast Read with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
		tData[5] = 0;  // Dummy clock
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI_Write(tData, 5);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI_Write(tData, 6);  // send read instruction along with the 32 bit memory address
	}

	SPI_Read(rData, size);  // Read the data
	csHIGH();  // pull the CS High
}



/*************************************************************************************************/


#define TIMEOUT 1000000  // Define a suitable timeout value
uint8_t txBuffer[256];
uint8_t rxBuffer[256];

void SPI_TransmitReceive_DMA(uint8_t* txData, uint8_t* rxData, uint16_t len)
{
	if(HAL_SPI_TransmitReceive_DMA(&hspi1, txData, rxData, len) != HAL_OK)
	{
		Error_Handler();
	}

	 // Wait for the DMA transfer to complete with timeout
	    uint32_t timeout = TIMEOUT;
	    while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) && (timeout-- > 0)) {}
	    if (timeout == 0) {
	        // Handle timeout error
	        Error_Handler();
	    }
}


void W25Q_Reset_DMA (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
	csLOW();
//	SPI_Write(tData, 2);
//	SPI_TransmitReceive_DMA(tData, NULL, 2);
	HAL_SPI_TransmitReceive_DMA(&hspi1,tData, NULL, 2);
	csHIGH();
	W25Q_Delay(100);
}



uint32_t W25Q_ReadID_DMA (void)
{
//	uint8_t pData[2] = {0x9F,0};  // Read JEDEC ID
	uint8_t tData = 0x9F;  // Read JEDEC ID

	uint8_t rData[3];


//	csLOW();
////	SPI_Write(&tData, 1);
////	SPI_TransmitReceive_DMA(&tData, NULL, 1);
//	DebugPrintf("debug test\n");
//	if(HAL_SPI_TransmitReceive_DMA(&hspi1, &tData, NULL, 1)==HAL_OK)
//	{
//		DebugPrintf("ok. tdma\n");
//	}
////	HAL_SPI_TransmitReceive_DMA(&hspi1, pData, NULL, 2);
//	HAL_Delay(500);
////	SPI_Read(rData, 3);
////	SPI_TransmitReceive_DMA(NULL, rData, 3);
//	if(HAL_SPI_TransmitReceive_DMA(&hspi1,NULL, rData, 3) == HAL_OK)
//	{
//		DebugPrintf("ok. rdma\n");
//	}
//
//	csHIGH();


	csLOW();
	if(HAL_SPI_TransmitReceive_DMA(&hspi1, &tData, rData, 1) == HAL_OK)
	{
		DebugPrintf("ok. trdma\n");
	}
		csHIGH();


//	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
	return ((rData[2]<<16)|(rData[1]<<8)|rData[0]);
//	return rData;
}
