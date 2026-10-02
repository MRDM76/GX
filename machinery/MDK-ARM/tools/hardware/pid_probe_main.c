#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "motor.h"
#include "stm32f1xx_hal_iwdg.h"

typedef struct {
    uint32_t magic, command, status, fault;
    uint32_t channel, duration_ms;
    float duty_limit, setpoint, kp, ki, kd;
    int32_t encoder_sign;
    uint32_t sample_count, sample_bytes, capacity, reserved;
} Probe_Control;

typedef struct {
    uint32_t time_ms;
    int32_t delta_a, delta_b;
    float cps_a, cps_b, target, duty;
    uint32_t phase;
    uint32_t gpio_levels, edges_a, edges_b;
} Probe_Sample;

typedef char Control_Size_Check[sizeof(Probe_Control) == 64 ? 1 : -1];
typedef char Sample_Size_Check[sizeof(Probe_Sample) == 44 ? 1 : -1];

volatile Probe_Control pidProbe;
volatile Probe_Sample pidTrace[320];
static IWDG_HandleTypeDef watchdog;
static PID_Controller controller;

void SystemClock_Config(void);

static HAL_StatusTypeDef set_probe_duty(Motor_HandleTypeDef *motor, float duty,
                                      uint32_t slow_decay)
{
    uint32_t mask, saved_udis, period, ticks;
    if (!slow_decay || duty <= 0.0f)
        return Motor_SetSpeed(motor, (int32_t)(duty + 0.5f));
    period = __HAL_TIM_GET_AUTORELOAD(motor->timer) + 1U;
    ticks = (period * (uint32_t)(duty + 0.5f) + 500U) / 1000U;
    mask = __get_PRIMASK();
    __disable_irq();
    saved_udis = motor->timer->Instance->CR1 & TIM_CR1_UDIS;
    SET_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in1_channel, period);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->in2_channel, period - ticks);
    if (saved_udis == 0U) CLEAR_BIT(motor->timer->Instance->CR1, TIM_CR1_UDIS);
    __set_PRIMASK(mask);
    return HAL_OK;
}

static void stop_motors(void)
{
    (void)Motor_Stop(&motorA);
    (void)Motor_Stop(&motorB);
}

void Error_Handler(void)
{
    stop_motors();
    pidProbe.fault = 2U;
    pidProbe.status = 4U;
    while (1) {}
}

static void run_probe(uint32_t mode)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t channel = pidProbe.channel;
    uint32_t duration = pidProbe.duration_ms;
    float limit = pidProbe.duty_limit;
    float setpoint = pidProbe.setpoint;
    int32_t sign = pidProbe.encoder_sign;
    PID_Config config = {0};
    Motor_HandleTypeDef *motor;
    uint32_t start, now, elapsed, previous, last_movement;
    float duty = 0.0f;
    uint32_t fault = 0U;
    uint32_t levels, previous_levels, changed;
    uint32_t edges[4] = {0};
    uint32_t flags = pidProbe.reserved;
    stop_motors();
    pidProbe.sample_count = 0U;
    pidProbe.fault = 0U;
    if ((mode != 1U && mode != 2U) || channel > 1U ||
        duration < 100U || duration > 2000U ||
        !(limit > 0.0f && limit <= 300.0f) ||
        !(setpoint > 0.0f && setpoint < 50000.0f) ||
        (mode == 1U && setpoint > limit) || (sign != 1 && sign != -1))
    {
        pidProbe.fault = 1U;
        pidProbe.status = 4U;
        return;
    }
    config.kp = pidProbe.kp;
    config.ki = pidProbe.ki;
    config.kd = pidProbe.kd;
    config.output_max = limit;
    config.integral_max = limit;
    config.derivative_tau = 0.03f;
    if (mode == 2U && !PID_Init(&controller, &config))
    {
        pidProbe.fault = 1U;
        pidProbe.status = 4U;
        return;
    }
    motor = channel == 0U ? &motorA : &motorB;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = flags & 1U ? GPIO_PULLUP : GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_Init(GPIOB, &gpio);
    previous_levels = ((GPIOA->IDR >> 6) & 3U) | ((GPIOB->IDR >> 4) & 12U);
    start = HAL_GetTick();
    previous = start;
    last_movement = start + 200U;
    pidProbe.status = 2U;
    while (1)
    {
        HAL_StatusTypeDef sample_status;
        uint32_t phase;
        uint32_t pin_index;
        int32_t delta;
        float measurement;
        HAL_IWDG_Refresh(&watchdog);
        levels = ((GPIOA->IDR >> 6) & 3U) | ((GPIOB->IDR >> 4) & 12U);
        changed = levels ^ previous_levels;
        for (pin_index = 0U; pin_index < 4U; ++pin_index)
            if (changed & (1U << pin_index)) ++edges[pin_index];
        previous_levels = levels;
        now = HAL_GetTick();
        elapsed = now - start;
        if (pidProbe.command == 3U)
        {
            pidProbe.command = 0U;
            fault = 3U;
            break;
        }
        if (elapsed >= duration + 200U)
        {
            duty = 0.0f;
            stop_motors();
        }
        if (elapsed >= duration + 700U) break;
        sample_status = Motor_Update();
        if (sample_status == HAL_BUSY) continue;
        if (sample_status != HAL_OK)
        {
            fault = 4U;
            break;
        }
        phase = elapsed < 200U ? 0U :
                (elapsed < duration + 200U ? 1U : 2U);
        delta = channel == 0U ? encoderA.delta : encoderB.delta;
        measurement = (channel == 0U ? encoderA.counts_per_second :
                       encoderB.counts_per_second) * (float)sign;
        if (phase == 1U)
        {
            if (delta != 0) last_movement = now;
            if (now - last_movement >= 300U)
            {
                fault = 5U;
                break;
            }
            if (mode == 1U) duty = setpoint;
            else if (measurement < -100.0f ||
                     !PID_Update(&controller, setpoint, measurement,
                                 (float)(now - previous) * 0.001f, &duty))
            {
                fault = 6U;
                break;
            }
            if (set_probe_duty(motor, duty, flags & 2U) != HAL_OK)
            {
                fault = 7U;
                break;
            }
        }
        previous = now;
        if (pidProbe.sample_count < 320U)
        {
            uint32_t index = pidProbe.sample_count;
            pidTrace[index].time_ms = elapsed;
            pidTrace[index].delta_a = encoderA.delta;
            pidTrace[index].delta_b = encoderB.delta;
            pidTrace[index].cps_a = encoderA.counts_per_second;
            pidTrace[index].cps_b = encoderB.counts_per_second;
            pidTrace[index].target = phase == 1U && mode == 2U ? setpoint : 0.0f;
            pidTrace[index].duty = duty;
            pidTrace[index].phase = phase;
            pidTrace[index].gpio_levels = levels;
            pidTrace[index].edges_a = (edges[0] & 65535U) | (edges[1] << 16);
            pidTrace[index].edges_b = (edges[2] & 65535U) | (edges[3] << 16);
            __DMB();
            pidProbe.sample_count = index + 1U;
        }
        for (pin_index = 0U; pin_index < 4U; ++pin_index) edges[pin_index] = 0U;
    }
    stop_motors();
    pidProbe.fault = fault;
    pidProbe.status = fault == 0U ? 3U : 4U;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    if (Motor_SystemInit() != HAL_OK) Error_Handler();
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_32;
    watchdog.Init.Reload = 624U;
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) Error_Handler();
    pidProbe.magic = 0x50494431U;
    pidProbe.sample_bytes = sizeof(Probe_Sample);
    pidProbe.capacity = 320U;
    pidProbe.status = 1U;
    while (1)
    {
        uint32_t command;
        HAL_IWDG_Refresh(&watchdog);
        (void)Motor_Update();
        command = pidProbe.command;
        if (command != 0U)
        {
            pidProbe.command = 0U;
            if (command == 3U) stop_motors();
            else run_probe(command);
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) Error_Handler();
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
