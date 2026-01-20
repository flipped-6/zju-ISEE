#ifndef _OLED_H_
#define _OLED_H_

#include "iic.h"
#include "font.h"
#include "main.h"


void Oled_Display_Char_Inv(u8 page,u8 col,char ch,u8 inv);
void Oled_Display_Num_with_Index(uint8_t* num,uint8_t index,uint8_t row,uint8_t col);

void OLED_Send_Command(u8 com);
void Oled_Write_Data(u8 data);
void Oled_Init(void);
void OLED_Clear(u8 clear_dat);
void Oled_Address(u8 row,u8 col);
void Oled_Display_Char(u8 page,u8 col,char ch);
void Oled_Display_String(u8 page,u8 col,char *str);
void Oled_Display_Pic(u8 high,u8 wight,u8 page,u8 col,u8 *str);

#endif
