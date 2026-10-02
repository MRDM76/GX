#ifndef MOTOR_UART_H
#define MOTOR_UART_H

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef Motor_UARTInit(void);
void Motor_UARTPoll(void);

#endif
