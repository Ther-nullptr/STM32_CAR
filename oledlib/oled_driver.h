#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H

//#include "stm32f10x.h"
#include "main.h"
#include "oled_basic.h"
#include "oled_config.h"

extern I2C_HandleTypeDef hi2c2;  //修改为所用对应的I2C
#define Scr12864_HI2C hi2c2

#define OLED_ADDRESS 0x78
#define OLED_WriteCom_Addr 0x00
#define OLED_WriteData_Addr 0x40

void OLED_Init(void);
void OLED_CLS(void);
void OLED_FILL(unsigned char BMP[]);

#endif