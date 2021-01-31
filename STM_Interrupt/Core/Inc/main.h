#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

void Error_Handler(void);

#define Out_Pin GPIO_PIN_5
#define Out_GPIO_Port GPIOA
#define EXTI_Pin GPIO_PIN_7
#define EXTI_GPIO_Port GPIOA

#endif /* __MAIN_H */
