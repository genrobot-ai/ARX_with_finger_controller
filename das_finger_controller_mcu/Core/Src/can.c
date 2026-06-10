/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "pid.h"

GripperLookupEntry gripper_lookup_table[207] = {
    {0.0f, 2.039772f},  // L=0.0
    {0.5f, 2.035183f},  // L=0.5
    {1.0f, 2.030514f},  // L=1.0
    {1.5f, 2.025763f},  // L=1.5
    {2.0f, 2.020933f},  // L=2.0
    {2.5f, 2.016023f},  // L=2.5
    {3.0f, 2.011035f},  // L=3.0
    {3.5f, 2.005971f},  // L=3.5
    {4.0f, 2.000830f},  // L=4.0
    {4.5f, 1.995613f},  // L=4.5
    {5.0f, 1.990321f},  // L=5.0
    {5.5f, 1.984956f},  // L=5.5
    {6.0f, 1.979517f},  // L=6.0
    {6.5f, 1.974005f},  // L=6.5
    {7.0f, 1.968422f},  // L=7.0
    {7.5f, 1.962767f},  // L=7.5
    {8.0f, 1.957043f},  // L=8.0
    {8.5f, 1.951248f},  // L=8.5
    {9.0f, 1.945385f},  // L=9.0
    {9.5f, 1.939453f},  // L=9.5
    {10.0f, 1.933454f},  // L=10.0
    {10.5f, 1.927387f},  // L=10.5
    {11.0f, 1.921254f},  // L=11.0
    {11.5f, 1.915055f},  // L=11.5
    {12.0f, 1.908791f},  // L=12.0
    {12.5f, 1.902462f},  // L=12.5
    {13.0f, 1.896070f},  // L=13.0
    {13.5f, 1.889614f},  // L=13.5
    {14.0f, 1.883095f},  // L=14.0
    {14.5f, 1.876514f},  // L=14.5
    {15.0f, 1.869871f},  // L=15.0
    {15.5f, 1.863167f},  // L=15.5
    {16.0f, 1.856402f},  // L=16.0
    {16.5f, 1.849577f},  // L=16.5
    {17.0f, 1.842692f},  // L=17.0
    {17.5f, 1.835748f},  // L=17.5
    {18.0f, 1.828746f},  // L=18.0
    {18.5f, 1.821685f},  // L=18.5
    {19.0f, 1.814567f},  // L=19.0
    {19.5f, 1.807391f},  // L=19.5
    {20.0f, 1.800159f},  // L=20.0
    {20.5f, 1.792870f},  // L=20.5
    {21.0f, 1.785526f},  // L=21.0
    {21.5f, 1.778126f},  // L=21.5
    {22.0f, 1.770671f},  // L=22.0
    {22.5f, 1.763162f},  // L=22.5
    {23.0f, 1.755598f},  // L=23.0
    {23.5f, 1.747981f},  // L=23.5
    {24.0f, 1.740310f},  // L=24.0
    {24.5f, 1.732587f},  // L=24.5
    {25.0f, 1.724811f},  // L=25.0
    {25.5f, 1.716983f},  // L=25.5
    {26.0f, 1.709104f},  // L=26.0
    {26.5f, 1.701173f},  // L=26.5
    {27.0f, 1.693191f},  // L=27.0
    {27.5f, 1.685158f},  // L=27.5
    {28.0f, 1.677076f},  // L=28.0
    {28.5f, 1.668943f},  // L=28.5
    {29.0f, 1.660761f},  // L=29.0
    {29.5f, 1.652529f},  // L=29.5
    {30.0f, 1.644249f},  // L=30.0
    {30.5f, 1.635921f},  // L=30.5
    {31.0f, 1.627544f},  // L=31.0
    {31.5f, 1.619119f},  // L=31.5
    {32.0f, 1.610647f},  // L=32.0
    {32.5f, 1.602128f},  // L=32.5
    {33.0f, 1.593562f},  // L=33.0
    {33.5f, 1.584949f},  // L=33.5
    {34.0f, 1.576290f},  // L=34.0
    {34.5f, 1.567585f},  // L=34.5
    {35.0f, 1.558834f},  // L=35.0
    {35.5f, 1.550038f},  // L=35.5
    {36.0f, 1.541197f},  // L=36.0
    {36.5f, 1.532311f},  // L=36.5
    {37.0f, 1.523380f},  // L=37.0
    {37.5f, 1.514405f},  // L=37.5
    {38.0f, 1.505386f},  // L=38.0
    {38.5f, 1.496324f},  // L=38.5
    {39.0f, 1.487218f},  // L=39.0
    {39.5f, 1.478068f},  // L=39.5
    {40.0f, 1.468876f},  // L=40.0
    {40.5f, 1.459641f},  // L=40.5
    {41.0f, 1.450363f},  // L=41.0
    {41.5f, 1.441043f},  // L=41.5
    {42.0f, 1.431682f},  // L=42.0
    {42.5f, 1.422278f},  // L=42.5
    {43.0f, 1.412833f},  // L=43.0
    {43.5f, 1.403346f},  // L=43.5
    {44.0f, 1.393818f},  // L=44.0
    {44.5f, 1.384250f},  // L=44.5
    {45.0f, 1.374640f},  // L=45.0
    {45.5f, 1.364991f},  // L=45.5
    {46.0f, 1.355301f},  // L=46.0
    {46.5f, 1.345570f},  // L=46.5
    {47.0f, 1.335800f},  // L=47.0
    {47.5f, 1.325990f},  // L=47.5
    {48.0f, 1.316141f},  // L=48.0
    {48.5f, 1.306252f},  // L=48.5
    {49.0f, 1.296324f},  // L=49.0
    {49.5f, 1.286358f},  // L=49.5
    {50.0f, 1.276352f},  // L=50.0
    {50.5f, 1.266308f},  // L=50.5
    {51.0f, 1.256225f},  // L=51.0
    {51.5f, 1.246104f},  // L=51.5
    {52.0f, 1.235945f},  // L=52.0
    {52.5f, 1.225748f},  // L=52.5
    {53.0f, 1.215513f},  // L=53.0
    {53.5f, 1.205240f},  // L=53.5
    {54.0f, 1.194929f},  // L=54.0
    {54.5f, 1.184581f},  // L=54.5
    {55.0f, 1.174196f},  // L=55.0
    {55.5f, 1.163774f},  // L=55.5
    {56.0f, 1.153314f},  // L=56.0
    {56.5f, 1.142818f},  // L=56.5
    {57.0f, 1.132285f},  // L=57.0
    {57.5f, 1.121714f},  // L=57.5
    {58.0f, 1.111108f},  // L=58.0
    {58.5f, 1.100465f},  // L=58.5
    {59.0f, 1.089785f},  // L=59.0
    {59.5f, 1.079069f},  // L=59.5
    {60.0f, 1.068317f},  // L=60.0
    {60.5f, 1.057528f},  // L=60.5
    {61.0f, 1.046704f},  // L=61.0
    {61.5f, 1.035843f},  // L=61.5
    {62.0f, 1.024947f},  // L=62.0
    {62.5f, 1.014015f},  // L=62.5
    {63.0f, 1.003046f},  // L=63.0
    {63.5f, 0.992042f},  // L=63.5
    {64.0f, 0.981003f},  // L=64.0
    {64.5f, 0.969927f},  // L=64.5
    {65.0f, 0.958816f},  // L=65.0
    {65.5f, 0.947669f},  // L=65.5
    {66.0f, 0.936487f},  // L=66.0
    {66.5f, 0.925269f},  // L=66.5
    {67.0f, 0.914016f},  // L=67.0
    {67.5f, 0.902727f},  // L=67.5
    {68.0f, 0.891402f},  // L=68.0
    {68.5f, 0.880041f},  // L=68.5
    {69.0f, 0.868646f},  // L=69.0
    {69.5f, 0.857214f},  // L=69.5
    {70.0f, 0.845747f},  // L=70.0
    {70.5f, 0.834244f},  // L=70.5
    {71.0f, 0.822706f},  // L=71.0
    {71.5f, 0.811131f},  // L=71.5
    {72.0f, 0.799521f},  // L=72.0
    {72.5f, 0.787875f},  // L=72.5
    {73.0f, 0.776193f},  // L=73.0
    {73.5f, 0.764475f},  // L=73.5
    {74.0f, 0.752721f},  // L=74.0
    {74.5f, 0.740931f},  // L=74.5
    {75.0f, 0.729104f},  // L=75.0
    {75.5f, 0.717241f},  // L=75.5
    {76.0f, 0.705342f},  // L=76.0
    {76.5f, 0.693405f},  // L=76.5
    {77.0f, 0.681432f},  // L=77.0
    {77.5f, 0.669422f},  // L=77.5
    {78.0f, 0.657375f},  // L=78.0
    {78.5f, 0.645290f},  // L=78.5
    {79.0f, 0.633169f},  // L=79.0
    {79.5f, 0.621009f},  // L=79.5
    {80.0f, 0.608812f},  // L=80.0
    {80.5f, 0.596576f},  // L=80.5
    {81.0f, 0.584302f},  // L=81.0
    {81.5f, 0.571990f},  // L=81.5
    {82.0f, 0.559639f},  // L=82.0
    {82.5f, 0.547248f},  // L=82.5
    {83.0f, 0.534819f},  // L=83.0
    {83.5f, 0.522350f},  // L=83.5
    {84.0f, 0.509841f},  // L=84.0
    {84.5f, 0.497291f},  // L=84.5
    {85.0f, 0.484702f},  // L=85.0
    {85.5f, 0.472071f},  // L=85.5
    {86.0f, 0.459399f},  // L=86.0
    {86.5f, 0.446685f},  // L=86.5
    {87.0f, 0.433929f},  // L=87.0
    {87.5f, 0.421131f},  // L=87.5
    {88.0f, 0.408289f},  // L=88.0
    {88.5f, 0.395405f},  // L=88.5
    {89.0f, 0.382476f},  // L=89.0
    {89.5f, 0.369503f},  // L=89.5
    {90.0f, 0.356485f},  // L=90.0
    {90.5f, 0.343422f},  // L=90.5
    {91.0f, 0.330313f},  // L=91.0
    {91.5f, 0.317157f},  // L=91.5
    {92.0f, 0.303954f},  // L=92.0
    {92.5f, 0.290704f},  // L=92.5
    {93.0f, 0.277404f},  // L=93.0
    {93.5f, 0.264056f},  // L=93.5
    {94.0f, 0.250658f},  // L=94.0
    {94.5f, 0.237209f},  // L=94.5
    {95.0f, 0.223709f},  // L=95.0
    {95.5f, 0.210156f},  // L=95.5
    {96.0f, 0.196551f},  // L=96.0
    {96.5f, 0.182891f},  // L=96.5
    {97.0f, 0.169177f},  // L=97.0
    {97.5f, 0.155406f},  // L=97.5
    {98.0f, 0.141579f},  // L=98.0
    {98.5f, 0.127695f},  // L=98.5
    {99.0f, 0.113751f},  // L=99.0
    {99.5f, 0.099747f},  // L=99.5
    {100.0f, 0.085682f},  // L=100.0
    {100.5f, 0.071554f},  // L=100.5
    {101.0f, 0.057363f},  // L=101.0
    {101.5f, 0.043107f},  // L=101.5
    {102.0f, 0.028784f},  // L=102.0
    {102.5f, 0.014393f},  // L=102.5
    {103.0f, 0.000066f},  // L=103.0
};

float feedback_n;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
LKMotor_Receive MG4005Motor;
LKMOTOR_POSE_SRND MG4005_send_pose;
LKMOTOR_MORE_SRND MG4005_Send;

CAN_FilterTypeDef canFilter;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE BEGIN 2 */
		// 配置CAN过滤器
		canFilter.FilterBank = 0;
		canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
		canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
		canFilter.FilterIdHigh = 0x0000;
		canFilter.FilterIdLow = 0x0000;
		canFilter.FilterMaskIdHigh = 0x0000;
		canFilter.FilterMaskIdLow = 0x0000;
		canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
		canFilter.FilterActivation = ENABLE;
//		canFilter.SlaveStartFilterBank = 14;

		if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK)
		{
				Error_Handler();
		}

		// 启动CAN
		if (HAL_CAN_Start(&hcan) != HAL_OK)
		{
				Error_Handler();
		}

		// 激活CAN RX中断
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
				Error_Handler();
		}

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/**
  * @brief  根据电机旋转圈数获取夹爪张开距离
  * @param  revolutions: 电机旋转圈数
  * @retval 夹爪张开距离（mm），超出范围取边界值
  */
float revolutions_to_distance(float revolutions)
{
    // 获取表中的圈数范围
    float rev_min = gripper_lookup_table[GRIPPER_LOOKUP_TABLE_SIZE - 1].theta;  // 最小圈数在表尾
    float rev_max = gripper_lookup_table[0].theta;  // 最大圈数在表头

    // 边界检查并限制
    if (revolutions >= rev_max) {
        return GRIPPER_L_MIN;  // 圈数大，对应L小
    }
    else if (revolutions <= rev_min) {
        return GRIPPER_L_MAX;  // 圈数小，对应L大
    }

    // 在表中查找合适的区间（反向查找，因为圈数是递减的）
    for (int i = 0; i < GRIPPER_LOOKUP_TABLE_SIZE - 1; i++) {
        float rev1 = gripper_lookup_table[i].theta;
        float rev2 = gripper_lookup_table[i + 1].theta;

        // 因为圈数是递减的，所以 rev1 > rev2
        if (revolutions <= rev1 && revolutions >= rev2) {
            // 找到了区间，进行线性插值
            float L1 = gripper_lookup_table[i].L;
            float L2 = gripper_lookup_table[i + 1].L;

            // 线性插值计算L
            float ratio = (rev1 - revolutions) / (rev1 - rev2);
            return L1 + (L2 - L1) * ratio;
        }
    }

    // 如果没找到（理论上不应该发生）
    return GRIPPER_L_MIN;
}

// CAN发送函数 扭矩控制模式
void CAN_LKMotor_More(int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4)
{
	uint8_t date[8];

	
    TxHeader.StdId = 0x280;             // 设置标准ID
    TxHeader.DLC = 8;              			// 设置数据长度
		TxHeader.ExtId = 0;                  // 使用标准ID
		TxHeader.IDE = CAN_ID_STD;           // 标准帧
		TxHeader.RTR = CAN_RTR_DATA;         // 数据帧
		TxHeader.TransmitGlobalTime = DISABLE;
    
		MG4005_Send.iqControl_1 = iqControl_1;
     MG4005_Send.iqControl_2 = iqControl_2;
     MG4005_Send.iqControl_3 = iqControl_3;
     MG4005_Send.iqControl_4 = iqControl_4;

     date[0] = MG4005_Send.buff[0];
     date[1] = MG4005_Send.buff[1];
     date[2] = MG4005_Send.buff[2];
     date[3] = MG4005_Send.buff[3];
     date[4] = MG4005_Send.buff[4];
     date[5] = MG4005_Send.buff[5];
     date[6] = MG4005_Send.buff[6];
     date[7] = MG4005_Send.buff[7];
	
    // 发送消息
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, date, &TxMailbox);
}

/*速度闭环控制命令
*	上位机最大速度设置为了18000
*	上位机最大力矩设置100
* 代码中力矩给0会被上位机的100给接管
*/
void CAN_LKMotor_speed(uint32_t ID,int16_t iqControl,int32_t speed)
{
	uint8_t date[8];
	TxHeader.StdId = ID;             // 设置标准ID
	TxHeader.DLC = 8;              			// 设置数据长度
	TxHeader.ExtId = 0;                  // 使用标准ID
	TxHeader.IDE = CAN_ID_STD;           // 标准帧
	TxHeader.RTR = CAN_RTR_DATA;         // 数据帧
	TxHeader.TransmitGlobalTime = DISABLE;
	
	MG4005_send_pose.iqControl_1 = iqControl;
	MG4005_send_pose.iqControl_2 = speed;

	date[0] = 0xA2;
	date[1] = 0;
	date[2] = MG4005_send_pose.buff[0];
	date[3] = MG4005_send_pose.buff[1];
	
	date[4] = MG4005_send_pose.buff[2];
	date[5] = MG4005_send_pose.buff[3];
	date[6] = MG4005_send_pose.buff[4];
	date[7] = MG4005_send_pose.buff[5];
				
// 发送消息
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, date, &TxMailbox);
}


void RMD_Single_Lap_Position_Loop2(uint32_t ID, int16_t maxspeed, int32_t angle)
{
	uint8_t date[8];
	TxHeader.StdId = ID;             // 设置标准ID
	TxHeader.DLC = 8;              			// 设置数据长度
	TxHeader.ExtId = 0;                  // 使用标准ID
	TxHeader.IDE = CAN_ID_STD;           // 标准帧
	TxHeader.RTR = CAN_RTR_DATA;         // 数据帧
	TxHeader.TransmitGlobalTime = DISABLE;
	


		 MG4005_send_pose.iqControl_1 = maxspeed;
     MG4005_send_pose.iqControl_2 = angle;

     date[0] = 0xA4;
     date[1] = 0;
     date[2] = MG4005_send_pose.buff[0];
     date[3] = MG4005_send_pose.buff[1];
     date[4] = MG4005_send_pose.buff[2];
     date[5] = MG4005_send_pose.buff[3];
     date[6] = MG4005_send_pose.buff[4];
     date[7] = MG4005_send_pose.buff[5];
				
// 发送消息
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, date, &TxMailbox);
}




void CAN_LKSend_Motor_Stop(uint32_t ID)
{
	uint8_t date[8];
	TxHeader.StdId = ID;             // 设置标准ID
	TxHeader.DLC = 8;              			// 设置数据长度
	TxHeader.ExtId = 0;                  // 使用标准ID
	TxHeader.IDE = CAN_ID_STD;           // 标准帧
	TxHeader.RTR = CAN_RTR_DATA;         // 数据帧
	TxHeader.TransmitGlobalTime = DISABLE;
	
	
	 date[0] = 0x81;
	 date[1] = 0;
	 date[2] = 0;
	 date[3] = 0;
	 date[4] = 0;
	 date[5] = 0;
	 date[6] = 0;
	 date[7] = 0;
				
// 发送消息
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, date, &TxMailbox);

}

/**
  * @brief  计算D(n)的值
  * @param  n: 输入的自变量n
  * @retval D(n): 计算结果（浮点型）
  */
float calculate_D(float n)
{
    float n3 = n * n * n;  // 计算n3（避免重复计算，提升效率）
    float n2 = n * n;      // 计算n2
    
    // 代入公式计算
    float D = -4.3312f * n3 + 4.0424f * n2 - 39.9316f * n + DISTANCE_MAX*1000;
    
    return D;
}




int angle_first=0;//接受数据的圈数

/*瓴控接收数据*/
void CAN_LKMotor_Recive_Data(LKMotor_Receive *motor,uint8_t aData[])
{
		float n;
		float ANGLE_INC;
	
//			if(aData[0]== 0xA1) 		
//			{
				if(angle_first==0)                                    //angle_first
				{
					angle_first=1;
				}
				
				 motor->angle_last=motor->angle_abs;
				motor->iqControl.buff[0] = aData[0];
				motor->iqControl.buff[1] = aData[1];
        motor->iqControl.buff[2] = aData[2];
        motor->iqControl.buff[3] = aData[3];
        motor->iqControl.buff[4] = aData[4];
        motor->iqControl.buff[5] = aData[5];
        motor->iqControl.buff[6] = aData[6];
        motor->iqControl.buff[7] = aData[7];
				motor->angle_abs =motor->iqControl.encoder/8;	//最大8191
				motor->speed =motor->iqControl.speed;
					
				if(angle_first==1)                                   //angle_first
				{
					motor->angle_last=motor->angle_abs;
					angle_first=2;
				}
				motor->angle_err=motor->angle_abs-motor->angle_last;  
				if(motor->angle_err>5000)                          //5000
					motor->flag=motor->flag-1;
				if(motor->angle_err<-5000)
					motor->flag=motor->flag+1;

				 motor->angle_Inc =motor->angle_abs+8191*motor->flag;
				if(  ALLflag.MG_Reset == 1 )
				{
					
					n = (motor->angle_Inc - MG4005MotorInit )*  0.643f / ANGLE_MAX;		//转换成上位机的
					feedback_n = n;
					 ANGLE_INC = motor_turns_to_distance( n ) /1000;
					
					if(ANGLE_INC <0)
					{
						ANGLE_INC = 0 ;
					}
//					if(ANGLE_INC >DISTANCE_MAX)
//					{
//						ANGLE_INC = DISTANCE_MAX ;
//					}
					motor->Txangle.angle_Inc =ANGLE_INC;		//反转一下发出去
					
				}
				else
				{
					motor->Txangle.angle_Inc =  -66.66f;
				}
				
				
				 motor->angle_last=motor->angle_abs;
//			}
//			else
//			{
//				if(angle_first==0)                                    //angle_first
//				{
//					angle_first=1;
//				}
//				
//				 motor->angle_last=motor->angle_abs;
//				
//				motor->iqControl.buff[0] = aData[0];
//				motor->iqControl.buff[1] = aData[1];
//        motor->iqControl.buff[2] = aData[2];
//        motor->iqControl.buff[3] = aData[3];
//        motor->iqControl.buff[4] = aData[4];
//        motor->iqControl.buff[5] = aData[5];
//        motor->iqControl.buff[6] = aData[6];
//        motor->iqControl.buff[7] = aData[7];
//				motor->angle_abs =motor->iqControl.encoder;	//最大65535
//				motor->speed =motor->iqControl.speed;
//					
//				if(angle_first==1)                                   //angle_first
//				{
//					motor->angle_last=motor->angle_abs;
//					angle_first=2;
//				}
//				motor->angle_err=motor->angle_abs-motor->angle_last;  
//				if(motor->angle_err>50000)                          //50000
//					motor->flag=motor->flag-1;
//				if(motor->angle_err<-50000)
//					motor->flag=motor->flag+1;

//				 motor->angle_Inc =motor->angle_abs+65535*motor->flag;
//					
//				motor->Txangle.angle_Inc = -motor->angle_Inc* 0.1 / 131072;		//转换成上位机的
//				
//				 motor->angle_last=motor->angle_abs;
//			}

}



// CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 接收消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
			ALLflag.MG_Can_RST = 0;	//清零标志位
			 if(can_recv_count < 65535) can_recv_count++; 
					switch(RxHeader.StdId)
					{

							case 0x141:CAN_LKMotor_Recive_Data(&MG4005Motor,RxData);break;
						
						default :break;
					}
    }
}

/* USER CODE END 1 */
