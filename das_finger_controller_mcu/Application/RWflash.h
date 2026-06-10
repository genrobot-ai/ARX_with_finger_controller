#ifndef __RWFLASH_H_
#define __RWFLASH_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define DATA_SIZE             500 	//一个文件400个字节左右

// 最后三页地址
#define FLASH_PAGE_60_ADDR  0x0800F000
#define FLASH_PAGE_61_ADDR  0x0800F400
#define FLASH_PAGE_62_ADDR  0x0800F800

extern	uint8_t Lcamera_buffer[DATA_SIZE];	//左边镜头
extern	uint8_t Ccamera_buffer[DATA_SIZE];	//中间镜头
extern 	uint8_t Rcamera_buffer[DATA_SIZE];	//中间镜头
				

// 简单函数声明
void Flash_Write(uint32_t address, uint8_t *data );
void Flash_Read(uint8_t *buffer , uint32_t address);

#endif

