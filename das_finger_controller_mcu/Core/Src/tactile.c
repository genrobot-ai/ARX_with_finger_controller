#include "tactile.h"
#include <stdint.h>

uint16_t crc_tactile_R;
uint16_t crc_tactile_L;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t length);


/*		磁编码器的校验位
*			第一版没有校验位，商家说后续会添加,需要判断数据正确新然后才能将数据发出去
*/
uint8_t tactile_IsCRCOK(const uint8_t *data, uint16_t length)
{
		uint16_t crc = (data[length-1]<<8 | data[length-2] );
		 
		return (crc == crc16_modbus(data,length-2) ) ? 1 : 0;
}


 uint16_t crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t lsb = crc & 0x0001;
            crc >>= 1;
            if (lsb) crc ^= 0xA001;
        }
    }
    return crc;
}

/*	Txdata : 发送给上位机的
**	Rxdata ：接收到触觉传感器的数据
*/

void tactile_deal_with(uint8_t *Txdata,const uint8_t *Rxdata)
{
	if(Rxdata[0] == 0xFF && Rxdata[1] == 0xFF && Rxdata[499] == 0xFF)
	{
		uint16_t j =0;
		for(uint16_t i=0;i<500;)
		{
			if(Rxdata[i] == 0xFF)
			{
				i ++;
			}
			else if(Rxdata[i] != 0xFF )
			{
				
				
				Txdata[j]=(Rxdata[i] + Rxdata[i+1]) / 2 ;
				i += 2;
				j++;
				if(j>= 224) break;
			}
			
		}
	}
	
}


