/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
	
#pragma anon_unions

#pragma pack()
#pragma pack(1)

typedef struct
{

    int32_t angle;    
		int32_t target;			 //目标值
    int32_t angle_abs;    
    int32_t angle_last;  
    int32_t angle_err;    
    int32_t angle_target; 
		int32_t angle_Inc;
	  
    int16_t speed;        
    int16_t speed_last;  
    int16_t speed_target; 
    int16_t given_current;
    uint8_t Max_Torque_Current;	//需要设置的最大转矩电流  不需要大于256
		int32_t flag;
		int8_t speed_err;
    int8_t speed_inc;

    union
    {
        struct
        {
						int8_t    ID;								//反馈的命令字节
            int8_t 		temperature;			//电机温度			单位：1°C/LSB
            int16_t 	iq;								//电机输出功率	范围：-2048~2048 对应实际转矩电流范围：-33A~33A
            int16_t 	speed;						//电机转速			单位：1dps/LSB
            uint16_t  encoder;					//编码器位置值	范围：0~16383
        };
        uint8_t buff[8];
    } iqControl;
	
		union
		{
			struct
			{
				float angle_Inc;   		//发送给上位机的数据  换算成夹爪之间的距离
				uint32_t  length;			//接收上位机的长度	8 -> 4
				uint8_t		spacing_select;		//间距选择， 0为0.103 1为0.104
				uint8_t   torque_flag;	//值为1则用上位机发送的最大转矩，值为0则用我给的默认的值
				int16_t   torque_max;	//转矩最大值
				float  angle_target;	//接收上位机的数据	  SDK发过来的是距离单位是m，要跟电机本省的的数据进行PID闭环
				
			};
			uint8_t buff[16];
		}Txangle;		//需要发送给上位机
		

} LKMotor_Receive;


typedef struct {
    float L;      // 夹爪张开距离（mm）
    float theta;  // 电机旋转圈数
} GripperLookupEntry;


#define GRIPPER_LOOKUP_TABLE_SIZE 207
#define GRIPPER_L_MIN 0.0f
#define GRIPPER_L_MAX 103.0f
#define GRIPPER_L_STEP 0.5f

//电机数据发送
typedef union
{
    struct
    {
        int16_t 	iqControl_1;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
        int32_t 	iqControl_2;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
    };
    uint8_t buff[6];
} LKMOTOR_POSE_SRND;


//电机数据发送
typedef union
{
    struct
    {
        int16_t 	iqControl_1;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
        int16_t 	iqControl_2;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
        int16_t 	iqControl_3;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
        int16_t 	iqControl_4;			//控制电机的转矩电流输出 范围-2000~2000 对应实际转矩电流范围：-32A~32A 母线电流和电机的实际扭矩因不同电机而异
    };
    uint8_t buff[8];
} LKMOTOR_MORE_SRND;

extern LKMotor_Receive MG4005Motor;
extern LKMOTOR_MORE_SRND MG4005_Send;
extern uint8_t RxData[8];
extern float feedback_n;

void CAN_LKMotor_More(int16_t iqControl_1, int16_t iqControl_2, int16_t iqControl_3, int16_t iqControl_4);
void CAN_LKMotor_speed(uint32_t ID,int16_t iqControl,int32_t speed);
void RMD_Single_Lap_Position_Loop2(uint32_t ID, int16_t maxspeed, int32_t angle);
void CAN_LKSend_Motor_Stop(uint32_t ID);
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

