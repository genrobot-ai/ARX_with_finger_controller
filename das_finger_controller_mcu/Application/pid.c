#include "pid.h"
#include "can.h"
#include "kth71xx.h"
#include <stdio.h>    //第一步重定向
#include "can.h"
#include <math.h>

PID PID_4005_angle,
		PID_4005_speed;

ALL ALLflag; 
float ANGLE_MAX; 
float DISTANCE_MAX  =0.103f;
float target_n;

/***** 												KP				KI			Kd																	*****/ 
float motor_4005_angle[5] = {30.0f,    0.0f,     0,    999,  900000  	};			//15000
float motor_4005_speed[5] = {25.0f,    0.0f,     0,    999,  780000     };				//3000



/**
  * @brief  计算n值
  * @param  theta: θ值（弧度）
  * @retval n值
  */
float calculate_n(float theta) {
    // 计算平方根内的值
    float cos_theta = cosf(theta);
    float inner = 225.0f - (26.0f*cos_theta - 25.0f)*(26.0f*cos_theta - 25.0f);
    
    // 确保平方根参数非负
    if(inner < 0) inner = 0;
    
    float sqrt_val = sqrtf(inner);
    float sin_theta = sinf(theta);
    
    return (sqrt_val - 26.0f*sin_theta - 11.07f) / 10.0f;
}

/**
  * @brief  计算θ值
  * @param  L: 输入参数L
  * @retval θ值（弧度）
  */
float calculate_theta(float L) {
    // 第一项: 53.03度转为弧度
    float term1 = DEG_TO_RAD(53.03f);
    
    // 计算反余弦的参数
    float numerator = L/2.0f + 15.51f*cosf(DEG_TO_RAD(33.0f)) - 19.517f;
    float denominator = 62.94f;
    float acos_arg = numerator / denominator;
    
    // 反余弦项
    float term2 = acosf(acos_arg);
    
    return calculate_n(term1 - term2);
}

float motor_turns_to_distance(double motor_turns)
{
    float angle_rad = (76.49 + 0.25 * motor_turns * 360.0) * PI / 180.0;
    return (29.33 - 125.53 * cos(angle_rad)) * 2.0;
}

float distance_to_motor_turns(double distance)
{
    float cos_val = (29.33 - distance / 2.0) / 125.53;
    if (cos_val < -1.0 || cos_val > 1.0)
        return -1.0;
    float angle_deg = acos(cos_val) * 180.0 / PI;
    return (angle_deg - 76.49) / 90.0;
}


/**
 * @brief 将距离转换为角度（线性映射）
 * @param distance 输入距离，范围0到1.5
 * @return 对应的角度，范围0到40
 */
float distanceToAngle(float distance)
{
	switch(MG4005Motor.Txangle.spacing_select) 
	{
		case 0x00:  DISTANCE_MAX = 0.103; break;
		case 0x01:  DISTANCE_MAX = 0.102; break;
		case 0x02:  DISTANCE_MAX = 0.101; break;
		case 0x03:  DISTANCE_MAX = 0.100; break;
		case 0x04:  DISTANCE_MAX = 0.0998; break;
		default  :DISTANCE_MAX = 0.103; break;
	}
	float distance_L ,distance2;
	
	    // 确保输入在有效范围内
    if (distance < 0.0f) {
         distance2 =0.0f;  // 小于0时返回最小角度
    }
    else if (distance > 0.234f) {
         distance2 = 234.0f; // 大于1.5时返回最大角度
    }
		else 
		{
			distance2 = distance * 1000;
		}
	distance_L = distance_to_motor_turns( distance2);
		
		target_n = distance_L;

    
    // 线性映射公式：角度 = (距离 / 最大距离) * 最大角度
    return (distance_L / distance_to_motor_turns( 234)  ) * ANGLE_MAX ; 
}


/* Flash操作函数 */

/* 擦除指定Flash页 */
void Flash_ErasePage(uint32_t address) {
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages = 1;
    
    uint32_t error = 0;
    HAL_FLASHEx_Erase(&erase, &error);
    
    HAL_FLASH_Lock();
}


/* 写入float到Flash */
uint8_t Flash_WriteFloat(uint32_t value) 
{
    
    /* 解锁Flash */
    HAL_FLASH_Unlock();
    
    /* 擦除目标页 */
//    Flash_ErasePage();
    
    /* 写入数据（按32位写入） */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PAGE_63_ADDR , value) != HAL_OK)
    {
        /* 写入失败 */
        HAL_FLASH_Lock();    
        return 0; 
    }
    
    /* 锁定Flash */
    HAL_FLASH_Lock();
    
    return 1;  // 成功
}

/* 从Flash读取float */
uint32_t Flash_ReadFloat(void)
{
   return *(__IO uint16_t*)FLASH_PAGE_63_ADDR ;
}

void pid_initw(PID *pid,float kp,float ki,float kd,float L_limit,float l_limit) 
{
     pid->Kp = kp;//比例
     pid->Ki = ki;//积分
     pid->Kd = kd;//微分
	 pid->L_limit = L_limit;
	 pid->l_limit = l_limit;
	 
}
/*
*
*		kf：前馈控制参数	（在kp，ki，kd调好之后在给值）
*		err_limit：前馈控制的限幅
*/
float pid_calc(PID*pid,float get, float set,float kf,float err_limit)
{
	 pid ->kf = kf;
    pid->last_err = pid->err;
    pid->err = get - set;	//偏差
    pid->Integral_err += pid->err; //积分误差	
	 pid->target_val = set ;
 
	if(pid->Integral_err > pid->L_limit)
	{
		pid->Integral_err = pid->L_limit;
	}
		if(pid->Integral_err < -pid->L_limit)
	{
		pid->Integral_err = -pid->L_limit;
	}
	
		pid -> err_target =pid ->kf *( pid->target_val - pid->target_last);	//一阶前馈
	if(pid -> err_target > err_limit)
	{
		pid -> err_target = err_limit;
	}
	if(pid -> err_target < -err_limit)
	{
		pid -> err_target = -err_limit;
	}
	
	 pid->out = pid->Kp*pid->err+pid->Ki*pid->Integral_err +pid->Kd*(pid->err-pid->last_err) + pid -> err_target;
	
	pid->target_last = pid->target_val;
	
	if(pid->out > pid->l_limit)
	{
		pid->out = pid->l_limit;
	}
		if(pid->out < -pid->l_limit)
	{
		pid->out = -pid->l_limit;
	}

	return pid->out;
	
}



float pid_calcc(PID*pid,float get, double set)
{
	
    pid->last_err = pid->err;
    pid->err = get - set;	//偏差
    pid->Integral_err += pid->err; //积分误差	
	
 
	if(pid->Integral_err > pid->L_limit)
	{
		pid->Integral_err = pid->L_limit;
	}
		if(pid->Integral_err < -pid->L_limit)
	{
		pid->Integral_err = -pid->L_limit;
	}
	
	 pid->out = pid->Kp*pid->err+pid->Ki*pid->Integral_err +pid->Kd*(pid->err-pid->last_err);
	
	if(pid->out > pid->l_limit)
	{
		pid->out = pid->l_limit;
	}
		if(pid->out < -pid->l_limit)
	{
		pid->out = -pid->l_limit;
	}

	
	return pid->out;
	
}

int16_t value_limit(int16_t value, int16_t min_val, int16_t max_val) 
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
    

void pid_init_all(void)
{
//		if( MG4005Motor.iqControl.speed >6000 || MG4005Motor.iqControl.speed < -6000)
//		{
//		MG4005Motor.Max_Torque_Current = 27;
//		}
//		else if( MG4005Motor.iqControl.speed >4000  || MG4005Motor.iqControl.speed < -4000)
//		{
//		MG4005Motor.Max_Torque_Current = 26;
//		}
//		else
//		{


//		if(MG4005Motor.angle_Inc < (MG4005MotorInit +2000) &&  MG4005Motor.angle_Inc > (MG4005MotorInit -100)   && 	ALLflag.Motor_able == 1 && ALLflag.MG_Reset == 1  
//			&& ( ( MG4005Motor.target - MG4005Motor.angle_Inc ) > ( ANGLE_MAX / 2 ) ||   ( MG4005Motor.target - MG4005Motor.angle_Inc ) < 0 ) 
//			&&( MG4005Motor.iqControl.iq < 25  && MG4005Motor.iqControl.iq > -25 ) 		)
//		{
//			MG4005Motor.Max_Torque_Current = 27;
//			motor_4005_angle[0] =  30;
//			motor_4005_speed[0] =  16;
//			motor_4005_angle[4] = 900000; 
//			motor_4005_speed[4] = 18000; 
//		}
//		else if( MG4005Motor.iqControl.speed < 300  && MG4005Motor.iqControl.speed > -300  &&(MG4005Motor.iqControl.iq > 20  || MG4005Motor.iqControl.iq < -20 )  )
//		{
//			MG4005Motor.Max_Torque_Current = 24;
//			motor_4005_angle[4] = 10000; 
//			motor_4005_speed[4] = 10000; 
//		}
//		else if( MG4005Motor.iqControl.speed < 300  && MG4005Motor.iqControl.speed > -300  &&(MG4005Motor.iqControl.iq > 10  || MG4005Motor.iqControl.iq < -10 )  )
//		{
//			MG4005Motor.Max_Torque_Current = 25;
//			motor_4005_angle[4] = 30000; 
//			motor_4005_speed[4] = 30000; 
//		}
//		else if( MG4005Motor.iqControl.speed < 500  && MG4005Motor.iqControl.speed > -500  &&(MG4005Motor.iqControl.iq > 10  || MG4005Motor.iqControl.iq < -10 )  )
//		{
//			MG4005Motor.Max_Torque_Current = 25;
//			motor_4005_angle[4] = 50000; 
//			motor_4005_speed[4] = 18000; 
//		}
//		else
//		{
//			MG4005Motor.Max_Torque_Current = 25;
//			motor_4005_angle[0] =  30;
//			motor_4005_speed[0] =  16;
//			motor_4005_angle[4] = 900000; 
//			motor_4005_speed[4] = 18000; 
//		}

//		} 
	
	

  /*4005电机PID参数初始化*/
	pid_initw(&PID_4005_angle,motor_4005_angle[0],motor_4005_angle[1],motor_4005_angle[2],motor_4005_angle[3],motor_4005_angle[4]);
	pid_initw(&PID_4005_speed,motor_4005_speed[0],motor_4005_speed[1],motor_4005_speed[2],motor_4005_speed[3],motor_4005_speed[4]);
	MG4005Motor.Txangle.length = 4;   // ← 加这一行

}



float pid_angleC(float angle_abs,float Target)
{
	float angle_Target;
	if(angle_abs>(Target-4096)&&angle_abs<(Target+4096))
	{  
		angle_Target=angle_abs;
	}
	if(angle_abs<(Target-4096))
	{
		angle_Target=8191+angle_abs;
	}
	if(angle_abs>(Target+4096))
	{
		angle_Target=-(8191-angle_abs);
	}
	return angle_Target;
}




void pid_count(void)
{
	
		pid_calc(&PID_4005_angle, MG4005Motor.target ,MG4005Motor.angle_Inc, 			1,80);		//电机PID位置环   第三个参数由MG4005Motor.Txangle.angle_Inc改成：(int32_t)encoder_Angle.RxAngle_360 用磁编码器去控制电机转动
		pid_calc(&PID_4005_speed, PID_4005_angle.out ,MG4005Motor.iqControl.speed,1,80);  		//电机PID速度环
	
}

/*--------------------------------------------------------------------------------------------*/
