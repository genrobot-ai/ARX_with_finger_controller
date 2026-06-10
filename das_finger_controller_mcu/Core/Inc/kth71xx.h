#ifndef __KTH71XX_H
#define __KTH71XX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#pragma anon_unions

#pragma pack()
#pragma pack(1)

	
#define KTH71_READ_ANGLE                        0x00
#define KTH71_READ_REG                          0x11
#define KTH71_WRITE_REG                         0x33
	
typedef union
{
	struct
	{
		float RxAngle_360_last;
		float RxAngle_360;			//处理后磁编码器的数据数据
		uint8_t  CRCstate;
		uint16_t RxSPI_Angle;		//读取的原始数据
	};
	uint8_t buff[11];
} RxAngle_Encoder;

extern RxAngle_Encoder encoder_Angle;
	
float convert_to_angle(uint16_t sensor_value);
uint8_t KTH71_ReadAngle(uint16_t* pAngle);
uint8_t KTH71_ReadReg(uint8_t addr, uint8_t* pRegValue);
uint8_t KTH71_WriteReg(uint8_t addr, uint8_t dataw);
void KTH71_UnlockReg(void);
void KTH71_LockReg(void);
void KTH71_WriteRegToMTP(void);

#ifdef __cplusplus
}
#endif

#endif /* __KTH71XX_H */
