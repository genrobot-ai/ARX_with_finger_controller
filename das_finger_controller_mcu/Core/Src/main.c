/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include <stdio.h>    //第一步重定向
#include "kth71xx.h"
#include "tactile.h" 
#include "stdlib.h"
#include "RWflash.h"
#include "stm32f1xx_it.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int32_t volatile MG4005MotorInit = 0;
uint32_t Calibration_angle;
int8_t volatile Key_State;		//触碰开关的状态  默认是1
uint8_t i2c1_state;
float saved_value;
uint32_t time_Inc;
uint32_t uid[3];
char  msg[50];
uint16_t len ;
uint8_t zero_found = 0;
int32_t last_angle_for_stall = 0;
uint16_t can_recv_count = 0;  // CAN接收计数
uint8_t motor_has_moved = 0;  // 电机是否已经已经运动过
int32_t MG4005MotorMax = 0;
uint8_t max_found = 0;
uint32_t stall_start_time = 0;
uint32_t stall_count = 0;  // 改成uint32_t
//int32_t tim3_interrupt_count;
//int32_t last_systick_count;
//float measured_frequency;
//uint8_t measurement_ready;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 初始化DWT
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 获取微秒数
uint32_t Get_Microseconds(void) {
    return DWT->CYCCNT / (HAL_RCC_GetSysClockFreq() / 1000000);
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */   
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();			// 磁编码器
  MX_USART1_UART_Init();
  MX_CAN_Init();			// 电机通信
  MX_I2C1_Init();			// 触觉传感器
  MX_TIM3_Init();			// 50Hz主任务定时器
  MX_I2C2_Init();			// 触觉传感器
  MX_TIM2_Init();			// 30Hz相机触发定时器
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);	//定时器任务，50HZ
	HAL_TIM_Base_Start_IT(&htim2);	//相机脉冲 30HZ
	
	MG4005MotorInit = 0;
ALLflag.MG_Reset = 0;      // 先不置1，等找到零点
ALLflag.MG_Key_RST = 0;
ALLflag.Motor_able = 1;
 zero_found = 0;
	// 启动UART DMA接收
	HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
	// 使能IDLE中断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	
//	DWT_Init();
	
	/* 1. 上电时从Flash读取保存的值 */
	
	/* 使用理论值，不再做软件标定 */
ANGLE_MAX = ANGLE_MAX_INIT;
		
/******** 启动单字节DMA接收 ，目前不用****/
//	HAL_UART_Receive_DMA(&huart1, &Uart_RxData, 1);
//	// 开启UART全局中断
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
/**********************************************************************************************************/ 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		 
		pid_init_all();			// 初始化所有PID控制器
		pid_count(); 	//PID运行 
		
		    
		
/****读取触觉传感器的数据（测试代码，printf会加代码运行时间）******/
  
//		printf("\r\n00000000000000000000000000000000000000000000\r\n");
//		for(uint16_t i=1;i<=448;i++)
//		{
//				printf("%d  ",TxBuffer_all[i+23]);  
//				
//			if(i%5 ==0)
//			printf("\r\n");
//		} 
//		printf("\r\n00000000000000000000000000000000000000000000\r\n");
		
//	printf(" %d  %d  %d  %f \r\n",MG4005Motor.iqControl.iq  ,MG4005Motor.Max_Torque_Current ,MG4005Motor.iqControl.speed,PID_4005_speed.out);
		// 主循环中打印测量结果

//    static uint32_t last_print = 0;
//    
//    if (measurement_ready && (HAL_GetTick() - last_print > 1000)) {
//        printf("测量频率: %.6f Hz, 误差: %.6f Hz (%.4f%%)\n",
//               measured_frequency,
//               measured_frequency - 50.0f,
//               (measured_frequency - 50.0f) / 50.0f * 100.0f);
//        
//        measurement_ready = 0;
//        last_print = HAL_GetTick();
//    }

/**********************************************************************************************************/ 

		
/*******磁编码器复位逻辑(电机必须失能，目前不用了)*****/ 

/**************************************************************************************************************/ 

/******* 触碰开关 状态读取*****/ 


static uint8_t printed_once = 0;
    if(max_found == 1 && printed_once == 0)
    {
        printf("\r\n>>> ANGLE_MAX = %d, MotorInit = %d, MotorMax = %d <<<\r\n",
               (int)ANGLE_MAX, (int)MG4005MotorInit, (int)MG4005MotorMax);
        printed_once = 1;
    }
		

			//Key_State = HAL_GPIO_ReadPin(GPIOB,Key1_Pin);		//读取触碰开关的状态  后续需要加与电机配合的条件
       Key_State = 1;
		
/**************************************************************************************************************/ 


		
/***********UARSTDMA发送（只拿出来了长一点的部分）********************/ \
		
		if (ALLflag.MCUID_send == 1)
		{
			// 读取设备ID
			uid[0] = HAL_GetUIDw0();
			uid[1] = HAL_GetUIDw1();
			uid[2] = HAL_GetUIDw2();
			
//			printf("UID is valid: %08lX-%08lX-%08lX\n", uid[2], uid[1], uid[0]);
			len = sprintf(msg, "das\r\n%08lX%08lX%08lXdas\r\n", uid[2], uid[1], uid[0]);
			// 通过DMA发送
			UART_Transmit_DMA((uint8_t*)msg, len );
			ALLflag.MCUID_send = 0;
		}
								
//暂时屏蔽触觉夹爪数据，客户只需要电机数据 

//		if(ALLflag.Tim_50HZ > 100)			// 系统上电2秒后才发送触觉数据			//记得解除注释
//		{

//			crc_tactile_L = tactile_IsCRCOK(rx_buf_L,502);		//验证磁编码器的校验位
//			crc_tactile_R = tactile_IsCRCOK(rx_buf_R,502);
//			
//			if(crc_tactile_L == 1)
//			tactile_deal_with(&TxBuffer_all[23],&rx_buf_L[0]);
//			if(crc_tactile_R == 1) 
//			tactile_deal_with(&TxBuffer_all[247],&rx_buf_R[0]);
//		}	

			// ============ 100Hz 推送电机距离反馈 ============
		if(ALLflag.Tim_50HZ_flag == 1)
		{
			// 填充电机距离反馈数据到 TxBuffer_drive[10..13]
			TxBuffer_drive[10] = MG4005Motor.Txangle.buff[3];
			TxBuffer_drive[11] = MG4005Motor.Txangle.buff[2];
			TxBuffer_drive[12] = MG4005Motor.Txangle.buff[1];
			TxBuffer_drive[13] = MG4005Motor.Txangle.buff[0];
			
			// 100Hz 推送
			DMA_Send_Packet(TxBuffer_drive, TxBuffer_driveLength);
			
			ALLflag.Tim_50HZ_flag = 0;
		}
		
			
//		if(ALLflag.Tim_50HZ_flag  == 1 &&  ALLflag.Tim_30HZ_flag == 1  && ALLflag.all_send == 1) //50HZ和30HZ读取一次一起了
//		{
////			printf("%X", 0xFF);
//			
////			if(time_Inc %50 ==0)
////			printf("%u ms\n",Get_Microseconds());
////			time_Inc++;
//			
//			TxBuffer_all[10] =MG4005Motor.Txangle.buff[3];	//电机
//			TxBuffer_all[11] =MG4005Motor.Txangle.buff[2];
//			TxBuffer_all[12] =MG4005Motor.Txangle.buff[1];
//			TxBuffer_all[13] =MG4005Motor.Txangle.buff[0];
//			
//			if(crc_tactile_L == 1)
//			tactile_deal_with(&TxBuffer_all[23],&rx_buf_L[0]);
//			if(crc_tactile_R == 1) 
//			tactile_deal_with(&TxBuffer_all[247],&rx_buf_R[0]);
//				
//			DMA_Send_Packet(TxBuffer_all,TxBuffer_allLength );	
//			
//			
//			
//			ALLflag.Tim_50HZ_flag  = 0; 
//			ALLflag.Tim_30HZ_flag  = 0;
//		}
//		else if( ALLflag.all_send == 1 )
//		{
//			if(ALLflag.Tim_50HZ_flag  == 1 )	//50HZ读取一次
//			{
////				printf("%X", 0xFD);
//				
////				if(time_Inc %50 ==0)
////				printf("%u ms\n",Get_Microseconds());
////				time_Inc++;
//				
//				TxBuffer_drive[10] =MG4005Motor.Txangle.buff[3];	//电机
//				TxBuffer_drive[11] =MG4005Motor.Txangle.buff[2];
//				TxBuffer_drive[12] =MG4005Motor.Txangle.buff[1];
//				TxBuffer_drive[13] =MG4005Motor.Txangle.buff[0];
//				
//				
//				
//				DMA_Send_Packet(TxBuffer_drive,TxBuffer_driveLength);	
//				
//				ALLflag.Tim_50HZ_flag  = 0; 
//			}
//			else if(ALLflag.Tim_30HZ_flag == 1 )	//30HZ读取一次
//			{
////				printf("%X", 0xFC);
//				
//				
//				
//				if(crc_tactile_L == 1)
//				tactile_deal_with(&TxBuffer_tactile[10],&rx_buf_L[0]);
//				if(crc_tactile_R == 1) 
//				tactile_deal_with(&TxBuffer_tactile[234],&rx_buf_R[0]);
//				DMA_Send_Packet(TxBuffer_tactile,TxBuffer_tactileLength);	
//				
//				ALLflag.Tim_30HZ_flag  = 0; 
//			}
//			
//		}

/**********************************************************************************************************/ 							
		
		
/************过流检测(优先级大于上位机传递的目标值，为了保护设备)********/


if( can_recv_count > 50 && 
    MG4005Motor.iqControl.temperature != 0 &&
    MG4005Motor.iqControl.iq < -15 &&
    MG4005Motor.speed <= 10 && MG4005Motor.speed >= -10 &&
    abs(MG4005Motor.angle_Inc - last_angle_for_stall) < 5)
{
    if(stall_count == 0)
        stall_start_time = HAL_GetTick();  // 第一次满足条件，记录起始时间
    stall_count++;

    if(zero_found == 0 && (HAL_GetTick() - stall_start_time) > 1000)  // 持续1秒
    {
        MG4005MotorInit = MG4005Motor.angle_Inc;
        ALLflag.MG_Reset = 1;
        zero_found = 1;
        stall_count = 0;
			  stall_start_time = HAL_GetTick();  // ← 加这一行，重置计时
        MG4005Motor.target = MG4005MotorInit + 10000;
    }
    else if(zero_found == 1 && (HAL_GetTick() - stall_start_time) > 1000)
    {
        MG4005Motor.target = MG4005Motor.angle_Inc;
        stall_count = 0;
    }
}
else
{
    stall_count = 0;  // 条件不满足就清零，重新计时
    last_angle_for_stall = MG4005Motor.angle_Inc;
    if( zero_found == 1 &&
    MG4005Motor.iqControl.iq > 150 &&  // 只检测正电流！去掉 iq < -20
    MG4005Motor.speed <= 5 && MG4005Motor.speed >= -5 &&
    (HAL_GetTick() - stall_start_time) > 1000)
    {
       if(max_found == 0)
{
    MG4005MotorMax = MG4005Motor.angle_Inc;
    max_found = 1;
    MG4005Motor.target = MG4005MotorInit;
    
    // 如果是 CALIB 命令触发的，报告测量结果（不写 Flash）
    if(ALLflag.Auto_Calibration_Cmd == 1)
		{
        int32_t measured_range = MG4005MotorMax - MG4005MotorInit;
        int32_t theoretical_range = (int32_t)ANGLE_MAX_INIT;
        int32_t deviation = measured_range - theoretical_range;
        
        // 不写 Flash，只报告对比结果给机械同事评估装配质量
        len = sprintf(msg, "das\r\nCHECK_meas%d_theo%d_dev%ddas\r\n", 
                     (int)measured_range, (int)theoretical_range, (int)deviation);
        UART_Transmit_DMA((uint8_t*)msg, len);
        
        ALLflag.Auto_Calibration_Cmd = 0;
			}
	}
//        else
//        {
//            //MG4005Motor.target = MG4005Motor.angle_Inc;
//        }
    }
}

/**********************************************************************************************************/

/*******力矩按键检测复位**********/		

		if(ALLflag.Motor_Calibration_Mode == 1)		//标定电机量程
		{
			
			 Calibration_angle= abs(MG4005Motor.angle_Inc - MG4005MotorInit) ; 	//当遇到校准时使用
			
			if(ALLflag.MG_Reset == 0)
			{
					if(ALLflag.MG_direction ==0 )	//	向上走
					{
							ALLflag.MG4005_electricity =-60;
							if(Key_State == 0 )
							{
								MG4005MotorInit = MG4005Motor.angle_Inc+800;
								ALLflag.MG_Key_RST2 = 1;
								ALLflag.MG_direction = 1;	//向上走
							}
					}
					
					if(ALLflag.MG_direction ==1)	//向下走
					{
							if( Key_State == 0 )
							{
								ALLflag.MG4005_electricity =60;
							}
							else
							{
								ALLflag.MG4005_electricity =60;
							}
							
							if(ALLflag.MG_Key_RST2 == 1 ) 	//为了让电机离开限位点
							{
								if( Key_State == 1)
								{
									
									MG4005MotorInit = MG4005Motor.angle_Inc+30;
									ALLflag.MG_Key_RST2 = 0 ;
									
								}
							}
							if(Calibration_angle > ANGLE_MAX_INIT && Calibration_angle<19000)
							{
								if(MG4005Motor.speed< 30 && MG4005Motor.speed >-30)
								{
										ALLflag.MG_Reset = 1;
										ALLflag.MG_direction =0;
										ALLflag.Motor_Calibration_Flag = 1;
									
								}
							}
						
					}
			}

			
			/****将数据保存在flash中******/
			if (ALLflag.MG_Reset == 1 && ALLflag.Motor_Calibration_Mode == 1 && ALLflag.Motor_Calibration_Flag == 1) 
			{
				
				if(ALLflag.Tim_10HZ_Motor2 >15)
				{
					
					Flash_ErasePage( FLASH_PAGE_63_ADDR );	//擦除指定Flash页
					ALLflag.flash_Write_flag = Flash_WriteFloat(Calibration_angle);
					ANGLE_MAX= Flash_ReadFloat() -ANGLE_OFFSET; //将读取到的值给到最大值
					printf("Calibration OK !");		//校准成功
					ALLflag.Motor_Calibration_Flag = 0;
					
				}
				
			}
			
		}
		
		if(ALLflag.Motor_Calibration_Mode == 0)
		{
				ALLflag.MG4005_electricity =-60; 
				

//			if(ALLflag.MG_Reset == 0 && ALLflag.Motor_able == 1 && MG4005Motor.iqControl.temperature != 0 )	//开始电机复位流程
//			{
//				
//				if( Key_State == 0)
//				{
//					MG4005MotorInit = MG4005Motor.angle_Inc+800;	//测试出来是750
//					MG4005Motor.target = MG4005MotorInit;
//					ALLflag.MG_Key_RST = 1;
//					ALLflag.MG_Reset = 1;	//复位结束
//					
//				}
//				
//			}
//			
//			if(ALLflag.MG_Key_RST == 1 ) 	//为了让电机离开限位点
//			{
//				
//				if( Key_State == 1)
//				{
//					MG4005MotorInit = MG4005Motor.angle_Inc+30;
//					ALLflag.MG_Key_RST = 0 ;
//					
//				}

//			}
		}
		
/**************************************************************************************************/		
			
/**********************驱动电机*************************/	


// ========= 临时测试：验证电机方向 =========
//CAN_LKMotor_speed(0x141, 30, -7000);  // 正转小力矩
// CAN_LKMotor_speed(0x141, 24, -5000); // 反转（确认方向后换这个）
//goto MOTOR_DRIVE_END;  // 跳过下面所有逻辑
// ==========================================		
		
			if( ALLflag.Motor_Calibration_Mode == 1 )		//标定模式			
			{
				if(ALLflag.MG_Reset == 0 )
				{
						CAN_LKMotor_speed(0x141,200,ALLflag.MG4005_electricity*10000); 
				}
				if(ALLflag.MG_Reset == 1  || MG4005Motor.iqControl.temperature >80 )	//复位成功		//失能标志位 或者 电机温度保护
				{
					CAN_LKMotor_speed(0x141,1,0);
				}
			
			}
			else	if( ALLflag.Motor_Calibration_Mode == 0 )	//SDK控制模式				
			{
				if( ALLflag.Motor_able == 0  || MG4005Motor.iqControl.temperature >80 )	//失能标志位 或者 电机温度保护
				{
						CAN_LKMotor_speed(0x141,1,0);
					
					
				}
				else if(ALLflag.MG_Reset == 0 && ALLflag.Motor_able == 1)
				{
						// 清除PID积分，防止积分饱和
					PID_4005_angle.Integral_err = 0;
					PID_4005_speed.Integral_err = 0;
					CAN_LKMotor_speed(0x141, 150, -18000);  // 力矩100，速度-18000
				}
				else if( ALLflag.Motor_able == 1 && ALLflag.MG_Reset == 1 )
	{
			if(ALLflag.Auto_Calibration_Cmd == 0)
			{
					MG4005Motor.target = MG4005MotorInit + distanceToAngle(MG4005Motor.Txangle.angle_target);
			}
    
//			// ↓↓↓ 新增：限制最大速度 ↓↓↓
//			int16_t max_speed = 5000;   // 原来可能是 18000 之类，改小让电机变慢
//			if(PID_4005_speed.out > max_speed) PID_4005_speed.out = max_speed;
//			if(PID_4005_speed.out < -max_speed) PID_4005_speed.out = -max_speed;
//    // ↑↑↑ 新增结束 ↑↑↑
			
			// ===== 力矩可控功能 =====
    // 根据上位机传过来的开关决定用什么扭矩
    int16_t torque_limit;
    
    if(MG4005Motor.Txangle.buff[9] == 0x01)
    {
        // 上位机启用力矩控制
        // buff[10..11] 是大端 int16，注意字节序
        torque_limit = (int16_t)((MG4005Motor.Txangle.buff[11] << 8) | MG4005Motor.Txangle.buff[10]);
        
        // 安全限幅，防止上位机传太大的值
        if(torque_limit > 500) torque_limit = 500;   // 上限 500（约 1.67 N·m）
        if(torque_limit < 100) torque_limit = 100;   // 下限 100（约 0.13 N·m）
    }
    else
    {
        // 上位机不控制，用默认 150（约 1 N·m）
        torque_limit = 150;
    }
    
    

		CAN_LKMotor_speed(0x141, torque_limit, PID_4005_speed.out);
	}
				
			}
			
//			MOTOR_DRIVE_END:;  // 跳转目标
			
//			HAL_Delay(1);	//延时1000HZ不影响
/**************************************************************************************************/		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)			//  STM32CubeMX自动生成
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


	//定时器中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
    if(htim == &htim3)
    {
			

			
			
//        tim3_interrupt_count++;
//        
//        // 每50次中断（1秒）测量一次频率
//        if (tim3_interrupt_count % 50 == 0) {
//            uint32_t current_systick = HAL_GetTick();
//            uint32_t elapsed_ms = current_systick - last_systick_count;
//            
//            if (elapsed_ms > 0) {
//                measured_frequency = 50.0f / (elapsed_ms / 1000.0f);
//                measurement_ready = 1;
//            }
//            
//            last_systick_count = current_systick;
//					}
						
				ALLflag.Tim_50HZ ++;			// 50Hz计数器递增
				
				ALLflag.Tim_50HZ_flag = 1;			// 设置50Hz标志位
				if(ALLflag.Tim_50HZ % 5 == 0)			// 10Hz任务
				{
					ALLflag.Tim_10HZ_flag  = 1;
					

					
					if(ALLflag.MG_Reset == 1 && ALLflag.Motor_Calibration_Mode == 1 && ALLflag.Motor_Calibration_Flag == 1 )  	//		电机总里程校准
					{
						ALLflag.Tim_10HZ_Motor2++; 			// 复位计时器递增
					} 
					else 
					{
						ALLflag.Tim_10HZ_Motor2 = 0;			// 清零计时器
					}
					
					
				}
    }
		if(htim == &htim2)
    {
			//暂时屏蔽客户不需要
			
//			if(ALLflag.Tim_50HZ > 100 )	//上电2s内不读取数据 
//			{
//				if(HAL_I2C_Master_Receive(&hi2c1, SENSOR_ADDRESS_L, rx_buf_L, 502,1000) == HAL_OK){	i2c1_state = 1;  }		//读取左边触觉传感器的值  在最小系统板上面测不到，需要将触觉传感器屏蔽
//				if(HAL_I2C_Master_Receive(&hi2c2, SENSOR_ADDRESS_R, rx_buf_R, 502,1000) == HAL_OK){	i2c1_state = 1;  }		//读取右边触觉传感器的值
//			}
//			
			
			ALLflag.MG_Can_RST ++;			//CAN通信重启机制
			if(ALLflag.MG_Can_RST >4)
			{
				
				//ALLflag.MG_Reset = 0;
				//ALLflag.Motor_able = 0; 临时注释
				MX_CAN_Init();
				ALLflag.MG_Can_RST = 0;
			}
			
			
			ALLflag.Tim_30HZ_flag = 1 ;  
			
			HAL_GPIO_TogglePin(GPIOA, R_TRG_Pin);	//三个相机的30HZ脉冲
      HAL_GPIO_TogglePin(GPIOA, C_TRG_Pin);
			HAL_GPIO_TogglePin(GPIOA, L_TRG_Pin);
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{ 
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
