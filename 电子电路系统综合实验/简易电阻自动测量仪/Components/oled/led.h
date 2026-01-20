#ifndef _LED_H
#define _LED_H

#include "main.h"
#define LED_ON GPIOA->ODR |= (1 << 0);
#define LED_OFF GPIOA->ODR &= ~(1 << 0);
#define LED_TRUN GPIOA->ODR ^= (1 << 0);

void Led_Init(void);


#endif
