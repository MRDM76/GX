/* Host-only HAL double. Never add this directory to firmware include paths. */
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_TIM_CHANNEL_STATE_RESET, HAL_TIM_CHANNEL_STATE_READY,
               HAL_TIM_CHANNEL_STATE_BUSY } HAL_TIM_ChannelStateTypeDef;
typedef struct { uint32_t CR1, SMCR, ARR, CCR[4], CNT; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; uint32_t state[4]; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
extern TIM_TypeDef test_tim2, test_tim3, test_tim4;
extern uint32_t test_mask;
#define TIM2 (&test_tim2)
#define TIM3 (&test_tim3)
#define TIM4 (&test_tim4)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_CHANNEL_ALL 60U
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *, uint32_t);
#define TIM_CR1_UDIS 2U
#define TIM_CR1_DIR 16U
#define TIM_CR1_CMS 96U
#define TIM_SMCR_SMS 7U
#define TIM_SMCR_ECE 16384U
#define TIM_OCMODE_PWM1 96U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define __get_PRIMASK() test_mask
#define __disable_irq() (test_mask = 1U)
#define __set_PRIMASK(mask) (test_mask = (mask))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->CCR[(c)/4U] = (v))
#define __HAL_TIM_DISABLE_OCxPRELOAD(t,c) ((void)(t),(void)(c))
#define __HAL_TIM_ENABLE_OCxPRELOAD(t,c) ((void)(t),(void)(c))
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(const TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *, const TIM_OC_InitTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
#endif
