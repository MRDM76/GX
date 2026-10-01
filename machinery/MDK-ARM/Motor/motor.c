#include "motor.h"

static int channel_valid(uint32_t channel)
{
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
           channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4;
}

static int motor_valid(const Motor_HandleTypeDef *motor)
{
    return motor != NULL && motor->initialized && motor->timer != NULL;
}

/* Prevent an overflow from latching only one of the two new CCR values.
 * UDIS does not stop the counter. Preserve the caller's interrupt mask.
 */
static void write_pair(Motor_HandleTypeDef *motor, uint32_t in1, uint32_t in2)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t saved_udis;
    __disable_irq();
    saved_udis = motor->timer->Instance->CR1 & TIM_CR1_UDIS;
    SET_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in1_channel, in1);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in2_channel, in2);
    if (saved_udis == 0U)
    {
        CLEAR_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    }
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *motor,
                            TIM_HandleTypeDef *timer,
                            uint32_t in1_channel, uint32_t in2_channel)
{
    HAL_StatusTypeDef status;
    uint32_t arr;

    if (motor == NULL || timer == NULL || motor->initialized ||
        !channel_valid(in1_channel) || !channel_valid(in2_channel) ||
        in1_channel == in2_channel)
    {
        return HAL_ERROR;
    }
    if (timer->Instance != TIM2 && timer->Instance != TIM3 &&
        timer->Instance != TIM4)
    {
        return HAL_ERROR;
    }
    arr = __HAL_TIM_GET_AUTORELOAD(timer);
    if (arr < 1U || arr > 65534U ||
        (timer->Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS)) != 0U ||
        (timer->Instance->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U ||
        HAL_TIM_GetChannelState(timer, in1_channel) != HAL_TIM_CHANNEL_STATE_READY ||
        HAL_TIM_GetChannelState(timer, in2_channel) != HAL_TIM_CHANNEL_STATE_READY)
    {
        return HAL_ERROR;
    }

    /* Load zero immediately, even if other channels already run on TIM2.
     * Do not issue UG: that would disturb the other motor's PWM period.
     */
    __HAL_TIM_DISABLE_OCxPRELOAD(timer, in1_channel);
    __HAL_TIM_DISABLE_OCxPRELOAD(timer, in2_channel);
    __HAL_TIM_SET_COMPARE(timer, in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(timer, in2_channel, 0U);
    __HAL_TIM_ENABLE_OCxPRELOAD(timer, in1_channel);
    __HAL_TIM_ENABLE_OCxPRELOAD(timer, in2_channel);

    status = HAL_TIM_PWM_Start(timer, in1_channel);
    if (status != HAL_OK) return status;
    status = HAL_TIM_PWM_Start(timer, in2_channel);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(timer, in1_channel);
        return status;
    }

    motor->timer = timer;
    motor->in1_channel = in1_channel;
    motor->in2_channel = in2_channel;
    motor->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetSpeed(Motor_HandleTypeDef *motor, int32_t duty)
{
    uint32_t ticks;
    uint32_t magnitude;
    uint32_t arr;
    if (!motor_valid(motor)) return HAL_ERROR;
    arr = __HAL_TIM_GET_AUTORELOAD(motor->timer);
    if (arr < 1U || arr > 65534U) return HAL_ERROR;
    if (duty > 1000) duty = 1000;
    if (duty < -1000) duty = -1000;
    magnitude = (uint32_t)(duty < 0 ? -duty : duty);
    ticks = ((arr + 1U) * magnitude + 500U) / 1000U;
    write_pair(motor, duty > 0 ? ticks : 0U, duty < 0 ? ticks : 0U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor)
{
    if (!motor_valid(motor)) return HAL_ERROR;
    write_pair(motor, 0U, 0U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_Brake(Motor_HandleTypeDef *motor)
{
    uint32_t arr;
    if (!motor_valid(motor)) return HAL_ERROR;
    arr = __HAL_TIM_GET_AUTORELOAD(motor->timer);
    if (arr < 1U || arr > 65534U) return HAL_ERROR;
    write_pair(motor, arr + 1U, arr + 1U);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_DeInit(Motor_HandleTypeDef *motor)
{
    HAL_StatusTypeDef first;
    HAL_StatusTypeDef second;
    if (!motor_valid(motor)) return HAL_ERROR;
    __HAL_TIM_DISABLE_OCxPRELOAD(motor->timer, motor->in1_channel);
    __HAL_TIM_DISABLE_OCxPRELOAD(motor->timer, motor->in2_channel);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in2_channel, 0U);
    first = HAL_TIM_PWM_Stop(motor->timer, motor->in1_channel);
    second = HAL_TIM_PWM_Stop(motor->timer, motor->in2_channel);
    motor->initialized = 0U;
    motor->timer = NULL;
    return first != HAL_OK ? first : second;
}

#include <limits.h>
//霍尔计数和转速计算
void Encoder_Reset(Encoder_State *s, uint16_t counter)
{
    s->previous = counter;
    s->sign = 1;
    s->counts_per_turn = 0U;
    s->delta = 0;
    s->position = 0;
    s->counts_per_second = 0.0f;
    s->rpm = 0.0f;
    s->valid = 0U;
}

int Encoder_Sample(Encoder_State *s, uint16_t counter, uint32_t elapsed_ms)
{
    uint32_t difference;
    int32_t delta;
    if (s == 0 || elapsed_ms == 0U) return 0;
    difference = (uint16_t)(counter - s->previous);
    s->previous = counter;
    s->valid = 0U;
    s->delta = 0;
    s->counts_per_second = 0.0f;
    s->rpm = 0.0f;
    if (elapsed_ms > 100U || difference == 32768U) return 0;
    delta = difference < 32768U ? (int32_t)difference : (int32_t)difference - 65536;
    if (s->sign < 0) delta = -delta;
    if ((delta > 0 && s->position > INT64_MAX - delta) ||
        (delta < 0 && s->position < INT64_MIN - delta)) return 0;
    s->delta = delta;
    s->position += delta;
    s->counts_per_second = (float)delta * 1000.0f / (float)elapsed_ms;
    if (s->counts_per_turn != 0U)
        s->rpm = s->counts_per_second * 60.0f / (float)s->counts_per_turn;
    s->valid = 1U;
    return 1;
}
