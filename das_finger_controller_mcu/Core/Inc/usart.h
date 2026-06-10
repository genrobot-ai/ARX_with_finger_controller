/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define RX_BUFFER_SIZE 508
#define TX_BUFFER_SIZE 256

	/* 数据包协议定义 - 使用"das\r\n"作为帧头和帧尾 */
#define PACKET_HEADER_SIZE  5       // 帧头帧尾长度："das\r\n"共5字节
#define PACKET_HEADER       "das\r\n" // 帧头字符串
#define PACKET_END          "das\r\n" // 帧尾字符串
#define PACKET_MAX_LENGTH   1100     // 最大数据包长度（包含头尾）
#define PACKET_MIN_LENGTH   10      // 最小数据包长度（头+数据+尾，至少头尾各5字节）

extern uint8_t TxBuffer_tactile[];	
extern uint8_t TxBuffer_encoder[];
extern uint8_t TxBuffer_drive[];
extern uint8_t TxBuffer_all[];
	
extern uint16_t TxBuffer_tactileLength;
extern uint16_t TxBuffer_encoderLength;
extern uint16_t TxBuffer_driveLength;
extern uint16_t TxBuffer_allLength;

extern uint8_t rxBuffer[RX_BUFFER_SIZE];
extern uint8_t txBuffer[TX_BUFFER_SIZE];
extern uint16_t rxLength ;
extern uint16_t txLength ;

extern uint8_t Uart_RxData;	 		//测试电机时需要的指令
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
void UART_Transmit_DMA(uint8_t *data, uint16_t length);
HAL_StatusTypeDef DMA_Send_Packet(uint8_t* data, uint16_t length);
uint8_t is_mcuid_received(const uint8_t *data, uint16_t len);
	
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

