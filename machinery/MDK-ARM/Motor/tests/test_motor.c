#include <assert.h>
#include <stdio.h>
#include "motor.h"

TIM_TypeDef test_tim2, test_tim3, test_tim4;
uint32_t test_mask;
TIM_HandleTypeDef htim2 = {TIM2, {1, 1, 1, 1}};
TIM_HandleTypeDef htim3 = {TIM3, {1, 1, 1, 1}};
TIM_HandleTypeDef htim4 = {TIM4, {1, 1, 1, 1}};
static uint32_t tick;
static int fail_encoder;
static int encoder_running[2];
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t, uint32_t c)
{
    int index = t->Instance == TIM3 ? 0 : 1;
    (void)c;
    if (fail_encoder && index == 1) return HAL_ERROR;
    encoder_running[index] = 1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *t, uint32_t c)
{
    (void)c;
    encoder_running[t->Instance == TIM3 ? 0 : 1] = 0;
    return HAL_OK;
}
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
    assert(Motor_SetDutyPercent(30, -30) == HAL_ERROR);
    assert(Motor_Init(&motorA, &t, 0, 4) == HAL_OK);
    assert(Motor_SetDutyPercent(30, -30) == HAL_ERROR);
    assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 0);
    assert(Motor_Init(&motorB, &t, 12, 8) == HAL_OK);
    assert(Motor_SetDutyPercent(30, -30) == HAL_OK);
    assert(TIM2->CCR[0] == 1080 && TIM2->CCR[1] == 0);
    assert(TIM2->CCR[2] == 1080 && TIM2->CCR[3] == 0);
    assert(Motor_SetDutyPercent(INT32_MIN, INT32_MAX) == HAL_OK);
    assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 3600);
    assert(TIM2->CCR[2] == 0 && TIM2->CCR[3] == 3600);
    assert(Motor_SetDutyPercent(0, 0) == HAL_OK);
    assert((TIM2->CCR[0] | TIM2->CCR[1] | TIM2->CCR[2] | TIM2->CCR[3]) == 0);
    assert(Motor_DeInit(&motorA) == HAL_OK);
    assert(Motor_DeInit(&motorB) == HAL_OK);
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
    assert(Motor_DeInit(&a) == HAL_OK);
    assert(Motor_DeInit(&b) == HAL_OK);
    assert(Motor_Update() == HAL_ERROR);
    fail_encoder = 1;
    assert(Motor_SystemInit() == HAL_ERROR);
    assert(!motorA.initialized && !motorB.initialized && !encoder_running[0]);
    fail_encoder = 0;
    assert(Motor_SystemInit() == HAL_OK);
    assert(Motor_SystemInit() == HAL_BUSY);
    assert(Motor_SetDutyPercent(30, 30) == HAL_OK);
    assert(TIM2->CCR[0] == 1080 && TIM2->CCR[3] == 1080);
    assert(Motor_Update() == HAL_BUSY);
    TIM3->CNT = 10;
    TIM4->CNT = 65526;
    tick = 10;
    assert(Motor_Update() == HAL_OK);
    assert(encoderA.delta == 10 && encoderB.delta == -10);
    tick = 111;
    assert(Motor_Update() == HAL_TIMEOUT);
    puts("PASS: Motor_SystemInit rollback/retry, Motor_SetDutyPercent(30,30), Motor_Update timing and feedback");
    {
        PID_Config cfg = {2, 1, 0, 0, 600, 0, 600, 0.02f};
        assert(CSGO(30, 30) == HAL_ERROR);
        assert(CSGO(0, 0) == HAL_OK);
        assert(Motor_SetTargetRPM(&motorA, 100) == HAL_ERROR);
        assert(Motor_PIDConfigure(&motorA, 0, 1, &cfg) == HAL_ERROR);
        assert(Motor_PIDConfigure(&motorA, 600, 0, &cfg) == HAL_ERROR);
        assert(Motor_PIDConfigure(&motorA, 600, 1, &cfg) == HAL_OK);
        assert(CSGO(100, 100) == HAL_ERROR);
        assert(!Motor_PIDIsEnabled(&motorA));
        assert(Motor_PIDConfigure(&motorB, 600, 1, &cfg) == HAL_OK);
        assert(CSGO(100, -100) == HAL_OK);
        assert(CSGO(0, 100) == HAL_ERROR);
        assert(Motor_PIDIsEnabled(&motorA));
        assert(CSGO(100, -100) == HAL_OK);
        assert(Motor_SetTargetRPM(&motorA, -100) == HAL_ERROR);
        assert(Motor_PIDIsEnabled(&motorA));
        tick += 10;
        assert(Motor_Update() == HAL_OK);
        assert(TIM2->CCR[0] == 724 && TIM2->CCR[1] == 0);
        assert(TIM2->CCR[2] == 724 && TIM2->CCR[3] == 0);
        TIM3->CNT += 20; /* 200 RPM overspeed: no reverse torque */
        tick += 10;
        assert(Motor_Update() == HAL_OK);
        assert(TIM2->CCR[0] == 0 && TIM2->CCR[1] == 0);
        assert(Motor_SetTargetRPM(&motorA, 0) == HAL_OK);
        assert(!Motor_PIDIsEnabled(&motorA) && Motor_PIDIsEnabled(&motorB));
        assert(Motor_SetDutyPercent(30, 30) == HAL_OK);
        assert(!Motor_PIDIsEnabled(&motorA) && !Motor_PIDIsEnabled(&motorB));
        tick += 10;
        assert(Motor_Update() == HAL_OK);
        assert(TIM2->CCR[0] == 1080 && TIM2->CCR[3] == 1080);
        assert(Motor_SetTargetRPM(&motorA, 100) == HAL_OK);
        assert(Motor_SetTargetRPM(&motorB, 100) == HAL_OK);
        tick += 101;
        assert(Motor_Update() == HAL_TIMEOUT);
        assert(!Motor_PIDIsEnabled(&motorA) && !Motor_PIDIsEnabled(&motorB));
        assert((TIM2->CCR[0] | TIM2->CCR[1] | TIM2->CCR[2] | TIM2->CCR[3]) == 0);
        assert(Motor_SetTargetRPM(&motorA, 100) == HAL_OK);
        encoderA.counts_per_turn = 0;
        tick += 10;
        assert(Motor_Update() == HAL_ERROR && !Motor_PIDIsEnabled(&motorA));
        assert(Motor_PIDConfigure(&motorA, 600, -1, &cfg) == HAL_OK);
        tick = UINT32_MAX - 4U;
        assert(Motor_Update() == HAL_TIMEOUT);
        assert(Motor_SetTargetRPM(&motorA, 100) == HAL_OK);
        TIM3->CNT -= 10;
        tick = 5;
        assert(Motor_Update() == HAL_OK);
        assert(encoderA.rpm == 100.0f);
        assert(CSGO(0, 0) == HAL_OK);
        assert(!Motor_PIDIsEnabled(&motorA) && !Motor_PIDIsEnabled(&motorB));
        puts("PASS: RPM PID configuration, signed outputs, overspeed, manual takeover, timeout stop, calibration tamper, tick wrap");
    }
    return 0;
}
#endif
