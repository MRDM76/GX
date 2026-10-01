#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Two-input H-bridge: IN1 PWM / IN2 low = positive direction.
 * IN1 low / IN2 PWM = negative direction; both low = coast.
 * Both high = brake (AT8236 truth table).
 * Zero-initialize each handle. Call from one foreground context only.
 */
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t in1_channel;
    uint32_t in2_channel;
    uint8_t initialized;
} Motor_HandleTypeDef;

/* Call AFTER MX_TIMx_Init(). Timer must be TIM2/3/4, up-counting,
 * internally clocked, with ARR in [1, 65534]. CubeMX must configure both
 * channels as PWM mode 1, active high. Channels must be unused.
 * Starts both channels at zero duty; does not change PSC/ARR or GPIOs.
 */
HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *motor,
                            TIM_HandleTypeDef *timer,
                            uint32_t in1_channel, uint32_t in2_channel);

/* Signed duty in permille, clamped to [-1000, 1000]. This is open-loop
 * duty control, not measured RPM. 500 = 50%, -500 = reverse 50%.
 * Updates take effect at the next timer update event (within one period).
 * Before reversing a moving load, coast until it has slowed down.
 */
HAL_StatusTypeDef Motor_SetSpeed(Motor_HandleTypeDef *motor, int32_t duty);
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *motor);
HAL_StatusTypeDef Motor_Brake(Motor_HandleTypeDef *motor);
HAL_StatusTypeDef Motor_DeInit(Motor_HandleTypeDef *motor);

/* Four-edge counts, not single-channel pulses. Zero CPR means uncalibrated. */
typedef struct {
    uint16_t previous;
    int8_t sign;
    uint32_t counts_per_turn;
    int32_t delta;
    int64_t position;
    float counts_per_second;
    float rpm;
    uint8_t valid;
} Encoder_State;

void Encoder_Reset(Encoder_State *s, uint16_t counter);
/* Call frequently enough that actual movement is <32768 counts per sample.
 * Gap >100 ms or exactly half a counter range invalidates this sample.
 */
int Encoder_Sample(Encoder_State *s, uint16_t counter, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif
#endif
