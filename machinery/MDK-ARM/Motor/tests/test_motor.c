#include <assert.h>
#include <stdio.h>
#include "motor.h"

TIM_TypeDef test_tim2, test_tim3, test_tim4;
uint32_t test_mask;
static int fail_start = -1;

HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(const TIM_HandleTypeDef *t, uint32_t c)
{ return (HAL_TIM_ChannelStateTypeDef)t->state[c / 4U]; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *t, const TIM_OC_InitTypeDef *cfg, uint32_t c)
{ t->Instance->CCR[c / 4U] = cfg->Pulse; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c)
{
    if ((int)c == fail_start) return HAL_ERROR;
    t->state[c / 4U] = HAL_TIM_CHANNEL_STATE_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t, uint32_t c)
{ t->state[c / 4U] = HAL_TIM_CHANNEL_STATE_READY; return HAL_OK; }

#ifndef MOTOR_TEST_HAL_ONLY
int main(void)
{
    TIM_HandleTypeDef t = {TIM2, {1, 1, 1, 1}};
    Motor_HandleTypeDef a = {0}, b = {0}, duplicate = {0};
    assert(Motor_Stop(&a) == HAL_ERROR);
    assert(Motor_Init(NULL, &t, 0, 4) == HAL_ERROR);
    TIM2->ARR = 65535;
    assert(Motor_Init(&a, &t, 0, 4) == HAL_ERROR);
    TIM2->ARR = 3599;
    assert(Motor_Init(&a, &t, 0, 0) == HAL_ERROR);
    assert(Motor_Init(&a, &t, 0, 16) == HAL_ERROR);
    fail_start = TIM_CHANNEL_2;
    assert(Motor_Init(&a, &t, 0, 4) == HAL_ERROR);
    assert(!a.initialized && t.state[0] == HAL_TIM_CHANNEL_STATE_READY);
    fail_start = -1;
    assert(Motor_Init(&a, &t, 0, 4) == HAL_OK);
    assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 0);
    assert(Motor_Init(&duplicate, &t, 0, 4) == HAL_ERROR);
    assert(Motor_Init(&b, &t, 12, 8) == HAL_OK);
    assert(Motor_SetSpeed(&a, 500) == HAL_OK);
    assert(TIM2->CCR[0] == 1800 && TIM2->CCR[1] == 0);
    assert(Motor_SetSpeed(&b, -250) == HAL_OK);
    assert(TIM2->CCR[2] == 900 && TIM2->CCR[3] == 0);
    assert(TIM2->CCR[0] == 1800 && TIM2->CCR[1] == 0);
    assert(Motor_SetSpeed(&a, INT32_MIN) == HAL_OK);
    assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 3600);
    assert(Motor_SetSpeed(&a, INT32_MAX) == HAL_OK);
    assert(TIM2->CCR[0] == 3600 && TIM2->CCR[1] == 0);
    test_mask = 1;
    TIM2->CR1 = TIM_CR1_UDIS | 1U;
    assert(Motor_Brake(&a) == HAL_OK);
    assert(test_mask == 1 && TIM2->CR1 == (TIM_CR1_UDIS | 1U));
    assert(TIM2->CCR[0] == 3600 && TIM2->CCR[1] == 3600);
    test_mask = 0;
    TIM2->CR1 = 1;
    assert(Motor_Stop(&a) == HAL_OK);
    assert(test_mask == 0 && TIM2->CR1 == 1);
    assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 0);
    assert(Motor_DeInit(&a) == HAL_OK);
    assert(Motor_SetSpeed(&a, 500) == HAL_ERROR);
    assert(t.state[2] == HAL_TIM_CHANNEL_STATE_BUSY);
    assert(TIM2->CCR[2] == 900);
    assert(Motor_Init(&a, &t, 0, 4) == HAL_OK);
    puts("PASS: initialization, rollback, channel ownership, shared timer, duty limits, brake, coast, interrupt-mask preservation, reinitialization");
    return 0;
}
#endif
