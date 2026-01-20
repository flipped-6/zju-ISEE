#ifndef _BSP_KEY_H
#define _BSP_KEY_H
#include "stm32f1xx_hal.h"

uint8_t KEY_Scan();
#define KEY0 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)//KEY0按键PC8
#define KEY1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)//KEY1按键PC9
#define KEY2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)//KEY2按键PD2
#define WK_UP HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)//WKUP按键PA0

#define KEY_L KEY0
#define KEY_U KEY1
#define KEY_R KEY2
#define KEY_D KEY3

#define KEY0_PRES 1 //KEY0
#define KEY1_PRES 2 //KEY1
#define KEY2_PRES 3 //KEY2
#define WKUP_PRES 4 //WK UP

#define KEYL_PRES 1 //KEY0
#define KEYU_PRES 2 //KEY1
#define KEYR_PRES 3 //KEY2
#define KEYD_PRES 4 //WK UP


#endif