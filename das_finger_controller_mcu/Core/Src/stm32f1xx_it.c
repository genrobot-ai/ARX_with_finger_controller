/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include "usart.h"
#include "pid.h"
#include "kth71xx.h" 
#include "can.h"
#include <stdio.h>    //第一步重定向
#include <string.h>
#include "i2c.h"
#include "RWflash.h"
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t camera_length;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
		// 检测IDLE中断		//多字节							//串口中断位置
		if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
		{
				// 清除IDLE标志
				__HAL_UART_CLEAR_IDLEFLAG(&huart1);
				
				// 停止DMA接收
				HAL_UART_DMAStop(&huart1);
				
				// 计算接收到的数据长度
				rxLength = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
				
					
					if (rxBuffer[0] == '1' && rxBuffer[1] == '2' && rxBuffer[2] == '3' && rxBuffer[3] == '4' && rxLength == 4 )		//用于标定电机的量程 将标定值写入flash   进入标定之后必须断电重启
					{
							ALLflag.Motor_Calibration_Mode =1;	//标志位置1后不得清零
							ALLflag.MG_Reset = 0 ;
							ALLflag.MG_direction =0;
//							ALLflag.all_send = 0;
							
					}
					else if(is_mcuid_received(rxBuffer,rxLength) == 1)
					{
						ALLflag.MCUID_send = 1;
						
					}
					else if (rxBuffer[0] == 'C' && rxBuffer[1] == 'A' && rxBuffer[2] == 'L' && 
         rxBuffer[3] == 'I' && rxBuffer[4] == 'B' && rxLength == 5)
				{
						// 上位机发 "CALIB" 触发自动标定
						ALLflag.Auto_Calibration_Cmd = 1;
    
						// 重置标定相关状态，让电机重新跑找零+找最大
								ALLflag.MG_Reset = 0;
								zero_found = 0;
								max_found = 0;
				}
					else if(rxBuffer[0] =='c' && rxBuffer[1] =='a' && rxBuffer[2] =='m' && rxBuffer[3] =='e' && rxBuffer[4] =='r' &&
						rxBuffer[5] =='a'  ) 
					{
								if(rxBuffer[6] =='w')
								{
												if(rxBuffer[7] =='l')		//写入的时候写入总的字节大小
												{
														Flash_Write(FLASH_PAGE_60_ADDR,&rxBuffer[8]);
												}
												if(rxBuffer[7] =='c')
												{
														 Flash_Write(FLASH_PAGE_61_ADDR,&rxBuffer[8]);
												}
												if(rxBuffer[7] =='r')
												{
														 Flash_Write(FLASH_PAGE_62_ADDR,&rxBuffer[8]);
												}
									
								}
								if(rxBuffer[6] =='r')
								{
									
												if(rxBuffer[7] =='l')
												{
	
													Flash_Read(Lcamera_buffer,FLASH_PAGE_60_ADDR);
													camera_length =	Lcamera_buffer[0] <<8 | Lcamera_buffer[1];
													DMA_Send_Packet(Lcamera_buffer,DATA_SIZE);	 
												}
												if(rxBuffer[7] =='c')
												{
													Flash_Read(Ccamera_buffer,FLASH_PAGE_61_ADDR);
													camera_length =	Lcamera_buffer[0] <<8 | Lcamera_buffer[1];
													DMA_Send_Packet(Ccamera_buffer,DATA_SIZE);	
												}
												if(rxBuffer[7] =='r')
												{
													Flash_Read(Rcamera_buffer,FLASH_PAGE_62_ADDR);
													camera_length =	Lcamera_buffer[0] <<8 | Lcamera_buffer[1];
													DMA_Send_Packet(Rcamera_buffer,DATA_SIZE);	
												}
									
								}
						
					}
					else if(rxBuffer[0] =='d' && rxBuffer[1] =='a' && rxBuffer[2] =='s' && rxBuffer[3] ==0x0D && rxBuffer[4] ==0x0A &&
						rxBuffer[rxLength-5] =='d' && rxBuffer[rxLength-4] =='a' && rxBuffer[rxLength-3] =='s' && 
						rxBuffer[rxLength-2] ==0x0D && rxBuffer[rxLength-1] ==0x0A )
					{
						
						ALLflag.Motor_Calibration_Mode = 0;	//退出标定模式
						ALLflag.MG_direction = 0;						//标定方向清零
/*
*
*
*				目前只在用这条
*						
*/
						
						if(rxBuffer[PACKET_HEADER_SIZE] == 0x02  && ALLflag.Motor_Calibration_Mode == 0)		//多条  & 电机不在标定模式时可用
						{
//							ALLflag.all_send = 1;
							
							MG4005Motor.Txangle.buff [4] = rxBuffer[PACKET_HEADER_SIZE+5];	//获取数据长度
							MG4005Motor.Txangle.buff [5] = rxBuffer[PACKET_HEADER_SIZE+4];
							MG4005Motor.Txangle.buff [6] = rxBuffer[PACKET_HEADER_SIZE+3];
							MG4005Motor.Txangle.buff [7] = rxBuffer[PACKET_HEADER_SIZE+2];
							
							MG4005Motor.Txangle.buff [8] = rxBuffer[PACKET_HEADER_SIZE+6];	//夹爪之间的距离
							
							MG4005Motor.Txangle.buff [9] = rxBuffer[PACKET_HEADER_SIZE+7];	//输入扭矩最大值的开关
							
							MG4005Motor.Txangle.buff [10] = rxBuffer[PACKET_HEADER_SIZE+9];	//输入的扭矩最大值
							MG4005Motor.Txangle.buff [11] = rxBuffer[PACKET_HEADER_SIZE+8];
							
							for(int i=0;i<MG4005Motor.Txangle.length;i++)								//获取放入电机的数据
							{
								MG4005Motor.Txangle .buff[15-i] = rxBuffer[PACKET_HEADER_SIZE+10 + i];
							}
							
						
							
							if(rxBuffer[19] == 0x01 )		//复位磁编码器 ，复位编码器时需要给电机也失能，防止出现意外
							{
//									ALLflag.encoder_Reset = 1;
									ALLflag.Motor_able = 0;
							}
							else if(rxBuffer[20] == 0x00 )
							{
									ALLflag.Motor_able = 0;
							}
							else if(rxBuffer[20] == 0x01 )
							{
									ALLflag.Motor_able = 1;
							}
							
							TxBuffer_all[10] =MG4005Motor.Txangle.buff[3];	//电机
							TxBuffer_all[11] =MG4005Motor.Txangle.buff[2];
							TxBuffer_all[12] =MG4005Motor.Txangle.buff[1];
							TxBuffer_all[13] =MG4005Motor.Txangle.buff[0];
							

							DMA_Send_Packet(TxBuffer_all,TxBuffer_allLength);	
							
						}

						 
					}
				
				// 重新启动DMA接收
				HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
		}
		
		// 检查接收寄存器非空中断  接收单个字节目前不用
//    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
//    {
//        // 清除标志
//        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
//        
//        // 重新启动DMA接收下一个字节
//				HAL_UART_Receive_DMA(&huart1, &Uart_RxData, 1);   
//				ALLflag.current_limit = 0;
//    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
