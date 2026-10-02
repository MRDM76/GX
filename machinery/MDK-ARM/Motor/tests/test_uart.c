#include "uart/stm32f1xx_hal.h"
#include "motor_uart.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

unsigned int test_uart_instance, test_gpio_instance;
uint32_t test_mask;
Motor_HandleTypeDef motorA, motorB;
static UART_HandleTypeDef *uart_handle;
static uint8_t *rx_buffer;
static uint8_t *pending_tx;
static char last_tx[192];
static uint32_t tick;
static unsigned int configured, abort_count;
static float first_target, second_target;
static HAL_StatusTypeDef update_status = HAL_BUSY;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOA);
    assert(gpio->Pin == GPIO_PIN_9 || gpio->Pin == GPIO_PIN_10);
}
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority)
{ assert(irq == USART1_IRQn && priority == 5 && subpriority == 0); }
void HAL_NVIC_EnableIRQ(uint32_t irq) { assert(irq == USART1_IRQn); }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart)
{
    assert(uart->Instance == USART1 && uart->Init.BaudRate == 115200);
    uart_handle = uart;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == uart_handle && size == 1);
    rx_buffer = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == uart_handle && size < sizeof(last_tx));
    assert(pending_tx == NULL);
    pending_tx = data;
    memcpy(last_tx, data, size);
    last_tx[size] = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ assert(uart == uart_handle); ++abort_count; return HAL_OK; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *uart) { assert(uart == uart_handle); }
HAL_StatusTypeDef Motor_PIDConfigureCounts(Motor_HandleTypeDef *motor, int8_t sign, const PID_Config *config)
{
    assert((motor == &motorA || motor == &motorB) && sign == 1);
    assert(config == &Motor_MeasuredCountsPID);
    ++configured;
    return HAL_OK;
}
HAL_StatusTypeDef Motor_Update(void) { return update_status; }
HAL_StatusTypeDef CSGO(float first, float second)
{ first_target = first; second_target = second; return HAL_OK; }

static void receive(const char *text)
{
    while (*text)
    {
        *rx_buffer = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(uart_handle);
    }
}

static void drain_tx(void)
{
    unsigned int iteration;
    for (iteration = 0; iteration < 20; ++iteration)
    {
        if (pending_tx != NULL)
        {
            assert(strcmp((char *)pending_tx, last_tx) == 0);
            pending_tx = NULL;
            HAL_UART_TxCpltCallback(uart_handle);
        }
        Motor_UARTPoll();
    }
    assert(pending_tx == NULL);
}

int main(void)
{
    unsigned int count;
    assert(Motor_UARTInit() == HAL_ERROR);
    motorA.initialized = motorB.initialized = 1;
    assert(Motor_UARTInit() == HAL_OK && configured == 2);
    assert(Motor_UARTInit() == HAL_BUSY);
    Motor_UARTPoll();
    assert(strstr(last_tx, "READY") != NULL);
    drain_tx();
    tick = 10;
    receive("3060\n");
    assert(first_target == 0 && second_target == 0);
    Motor_UARTPoll();
    assert(fabsf(first_target - 30.30303f) < 0.0001f);
    assert(strstr(last_tx, "ACK RAW=3060") != NULL);
    drain_tx();
    HAL_UART_ErrorCallback(uart_handle);
    Motor_UARTPoll();
    assert(first_target == 0 && abort_count == 1);
    drain_tx();
    tick = 40;
    receive("\n0000\n9900\n");
    Motor_UARTPoll();
    assert(first_target == 100);
    drain_tx();
    for (count = 0; count < 70; ++count) receive("9");
    Motor_UARTPoll();
    assert(first_target == 0 && abort_count == 2);
    drain_tx();
    tick = 70;
    receive("\n0000\n9900\n");
    Motor_UARTPoll();
    drain_tx();
    tick = 570;
    Motor_UARTPoll();
    assert(first_target == 0 && strstr(last_tx, "LINK_TIMEOUT") != NULL);
    drain_tx();
    tick = 600;
    receive("\n0000\n9900\n");
    Motor_UARTPoll();
    drain_tx();
    update_status = HAL_TIMEOUT;
    Motor_UARTPoll();
    assert(first_target == 0);
    update_status = HAL_BUSY;
    drain_tx();
    tick = 630;
    receive("\n0000\n9900\n");
    tick = 730;
    Motor_UARTPoll();
    assert(first_target == 0);
    drain_tx();
    tick = 760;
    receive("\n0000\n");
    Motor_UARTPoll();
    drain_tx();
    receive("9900\n9900\n9900\n9900\n9900\n9900\n9900\n9900\n9900\n");
    Motor_UARTPoll();
    assert(first_target == 0);
    drain_tx();
    assert(strstr(last_tx, "TX_BACKPRESSURE") != NULL);
    puts("PASS: UART init, interrupt RX, stable async TX buffers, RX errors/overflow, stale input, motor/lease stop, TX backpressure");
    return 0;
}
