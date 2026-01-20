#include "BSP_KEY.h"
uint8_t KEY_Scan()
    {
        //static uint8_t key_up=1;//按键按松开标志
       // if(mode)key_up=1; //支持连按
        if((KEY0==0||KEY1==0||KEY2==0||WK_UP==0))
        {
        HAL_Delay(10);//去抖动

        if(KEY0==0)return KEY0_PRES;
        else if(KEY1==0)return KEY1_PRES;
        else if(KEY2==0)return KEY2_PRES;
        else if(WK_UP==0)return WKUP_PRES;
        }
        return 0;//无按键按下
    }