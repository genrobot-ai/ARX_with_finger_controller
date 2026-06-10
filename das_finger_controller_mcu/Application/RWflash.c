#include "RWflash.h"
#include "pid.h"

uint8_t Lcamera_buffer[DATA_SIZE];	//左边镜头
uint8_t Ccamera_buffer[DATA_SIZE];	//中间镜头
uint8_t Rcamera_buffer[DATA_SIZE];	//右边镜头
// 写入数据（最简单版） 三个相机标定数据写入
void Flash_Write(uint32_t address, uint8_t *data){
    // 1. 擦除最后一页
    Flash_ErasePage(address);
    
    // 2. 写入数据
    HAL_FLASH_Unlock();
    
    uint32_t addr = address;
    
    for(int i = 0; i < DATA_SIZE; i += 4) {
        // 组合4个字节
        uint32_t word = 0;
        for(int j = 0; j < 4 && (i+j) < DATA_SIZE; j++) {
            word |= (data[i+j] << (j * 8));
        }
        
        // 写入Flash
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);
        addr += 4;
    }
    
    HAL_FLASH_Lock();
}

// 读取数据	三个相机标定数据读出
void Flash_Read(uint8_t *buffer , uint32_t address) {
    uint8_t *flash_ptr = (uint8_t*)address;
    
    for(int i = 0; i < DATA_SIZE; i++) {
        buffer[i] = flash_ptr[i];
    }
}

