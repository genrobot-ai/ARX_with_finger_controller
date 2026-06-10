#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define ANGLE_MAX_INIT  (5267.0f)  //标定的最大距离				填到这里  5000  5238  5280 //设备2是5363
#define ANGLE_OFFSET		50			//标定的最大距离的补偿
// 常数定义
#define PI 3.14f
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)

extern float ANGLE_MAX;
extern float DISTANCE_MAX;		//标定的最大距离   


/* Flash最后四页地址定义 */
#define FLASH_PAGE_63_ADDR    0x0800FC00  // 最后一页的起始地址（最后一页1KB）
//#define FLASH_PAGE_SIZE    1024        // 1KB


/* 函数声明 */

uint8_t Flash_WriteFloat(uint32_t value);
uint32_t Flash_ReadFloat(void);
extern float target_n;
void Flash_ErasePage(uint32_t address);

typedef struct 
{ 
	float  Kp; 
	float  Ki; 
	float  Kd; 
	float kf;
	float err_target;
	float target_val;
	float target_last;
	
	float out;
  float last_out;
	
	float  L_limit; 
	float  l_limit; 
	float  err;
	float  Integral_err;    
	float  last_err;
	float  alpha;//不完全微分系数
	float  D_out;
	float  D_L_out;
	
} PID;

typedef struct
{
	uint32_t Tim_50HZ;			//定时器 0.02s   
	
	uint32_t Tim_10HZ_RST;			//定时器	 0.1s    用于磁编码器复位时间定时
	uint32_t Tim_10HZ_Motor;		//电机标定0位的计时
	
	
	uint16_t torque_max;
	
	uint8_t Tim_30HZ_flag;		//触觉传感器发送频率
	uint8_t all_send;		//发送多条（触觉 磁编码 电机）
	uint8_t MCUID_send;		//发送单片机的ID号
	uint8_t MG_Key_RST;
	uint8_t MG_Key_RST2;
	uint8_t current_limit;	//电机限流，夹取到东西之后电流太大置1，电机停到当前位置
	uint8_t MG_Reset;	//MG4005电机复位标志位 1代表复位成功
	uint8_t MG_direction;	//MG4005电机方向 0为往上（负的）， 1为往下（正的）
	uint8_t Tim_10HZ_Motor2;		//电机标定总量程
	
	uint8_t flash_Write_flag;	//flash写入成功为1;
	uint8_t MG_Can_RST;		//解决板子在电机上电之前上电无法控制的问题
	uint8_t Motor_Calibration_Mode;	//电机标定模式
	uint8_t Motor_Calibration_Flag;	//1：校准完成
	uint8_t Motor_able;	//电机使能  1为使能
//	uint8_t MG_Start_Reset;	//磁编码器开始复位标志位 1为开始复位
	int16_t MG4005_electricity;	//复位需要的电流值，夹取检测  1为检测到之
//	uint8_t encoder_Reset;		//编码器复位标志位  1：复位开始 复位完成后自动清零
//	uint8_t tactile_send;	//发送触觉传感器标志位，一次发两个的448字节   1的时候要发送

	uint8_t Auto_Calibration_Cmd;   // 上位机触发自动标定的标志位
	uint8_t Auto_Calibration_Done;  // 标定完成回传给上位机的标志位
	
	uint8_t Tim_50HZ_flag;
	uint8_t Tim_10HZ_flag;
	
	
}ALL;



extern float motor_4005_angle[5];
extern float motor_4005_speed[5];
	
extern ALL ALLflag;
extern  PID PID_4005_angle,
		    PID_4005_speed;
float motor_turns_to_distance(double motor_turns);
float distance_to_motor_turns(double distance);
float distanceToAngle(float distance);
void pid_initw(PID *pid,float kp,float ki,float kd,float L_limit,float l_limit);
float pid_calc(PID*pid,float get, float set,float kf,float err_limit);
void pid_init_all(void);
void pid_count(void);


#endif





