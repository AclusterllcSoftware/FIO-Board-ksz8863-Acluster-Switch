/*
 * ksz8863.c
 *
 *  Created on: May 14, 2024
 *      Author: Asus
 */
#include "ksz8863.h"
#include "main.h"
#include "debug.h"

extern SPI_HandleTypeDef hspi4;
#define KSZ_SPI hspi4

void ksz_csLOW (void)
{
	HAL_GPIO_WritePin (SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
}

void ksz_csHIGH (void)
{
	HAL_GPIO_WritePin (SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
}

void ksz_SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&KSZ_SPI, data, len, 2000);
}

void ksz_SPI_Read (uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&KSZ_SPI, data, len, 5000);
}


uint32_t ksz_ReadID(void)
{
	uint8_t tData[2];
	tData[0] = 0x03;
	tData[1] = 0x00;
	uint8_t rData[1] = {0};//[3];
	ksz_csLOW();
	ksz_SPI_Write(tData, 2);
	ksz_SPI_Read(rData, 1);
	ksz_csHIGH();
	return rData[0];
//	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}


uint8_t ksz_Read(void)
{
	uint8_t tData[2];
	tData[0] = 0x03;
	tData[1] = 0x01;
	uint8_t rData[1];//[3];
	ksz_csLOW();
	ksz_SPI_Write(tData, 2);
	ksz_SPI_Read(rData, 1);
	ksz_csHIGH();
	uint8_t p1 = rData[0];
	 DebugPrintf("chip id %X\r\n", p1);
	return rData[0];
//	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}


uint8_t ksz_write(void)
{
	uint8_t tData[2];
	uint8_t rData[1];//[3];


	tData[0] = 0x02;
	tData[1] = 0x01;
	tData[2] = 0x30;

	ksz_csLOW();
	ksz_SPI_Write(tData, 3);
//	ksz_SPI_Read(rData, 1);
	ksz_csHIGH();
//	uint8_t p1 = rData[0];
//	 DebugPrintf("chip id %X\r\n", p1);
	return rData[0];
//	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

/**************************************************************************************************/

uint8_t gtData[2];
uint8_t grData[2];
uint32_t ksz_ReadID_DMA(void)
{
//	uint8_t tData[2];
	gtData[0] = 0x03;
	gtData[1] = 0x00;
//	uint8_t rData[1] = {0};//[3];
//	uint8_t rData = 0;

//
//	ksz_csLOW();
//	HAL_Delay(200);
//	if(HAL_SPI_TransmitReceive_DMA(&hspi4, gtData, grData, 2) == HAL_OK)
//	{
////		return ((grData[0]<<8)|grData[1]);
//		HAL_Delay(1000);
//		ksz_csHIGH();
//		DebugPrintf("grData is %X\r\n", grData[0]);
//		return grData[0];
//	}
//	else
//	{
//		ksz_csHIGH();
//		return 2;
//	}

//	ksz_csLOW();
//	if(HAL_SPI_Transmit_DMA(&hspi4, gtData, 2)== HAL_OK)
//	{
//		if(HAL_SPI_Receive_DMA(&hspi4, grData, 1) == HAL_OK)
//		{
//			ksz_csHIGH();
////			return ((grData[0]<<8)|grData[1]);
//			return grData[1];
//		}
//		ksz_csHIGH();
//		return 5;
//	}
//	ksz_csHIGH();
//	return 3;


	ksz_csLOW();
	if(HAL_SPI_TransmitReceive_DMA(&hspi4, gtData, NULL, 2)==HAL_OK)
	{
		DebugPrintf("ok. tdma\n");
	}
	HAL_Delay(500);
	if(HAL_SPI_TransmitReceive_DMA(&hspi4,NULL, grData, 1) == HAL_OK)
	{
		DebugPrintf("ok. rdma\n");
	}

	ksz_csHIGH();

	return grData[0];
}


uint8_t ksz_Read_DMA(void)
{
//	uint8_t tData[2];
//	tData[0] = 0x03;
//	tData[1] = 0x01;
//	uint8_t rData[1];//[3];
//	ksz_csLOW();
//	ksz_SPI_Write(tData, 2);
//	ksz_SPI_Read(rData, 1);
//	ksz_csHIGH();
//	return rData[0];
////	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}



