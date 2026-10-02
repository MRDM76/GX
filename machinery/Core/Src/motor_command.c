#include "motor_command.h"
#include "motor.h"
#include <stdio.h>

static Motor_CommandSend send_reply;
static uint32_t last_byte_tick;
static uint32_t last_command_tick;
static uint8_t digits[4];
static uint8_t length;
static uint8_t discard;
static uint8_t fault_latched;
static uint8_t moving;

static void stop(void)
{
    (void)CSGO(0.0f, 0.0f);
    moving = 0U;
}

static void reply(const char *message)
{
    if (send_reply == NULL || !send_reply(message))
    {
        stop();
        fault_latched = 1U;
    }
}

void Motor_CommandFault(const char *reason)
{
    char message[MOTOR_COMMAND_REPLY_SIZE];
    stop();
    fault_latched = 1U;
    length = 0U;
    discard = 1U;
    (void)snprintf(message, sizeof(message), "ERR %s M1=0 M2=0 UNIT=PERCENT\r\n", reason);
    reply(message);
}

void Motor_CommandInit(Motor_CommandSend send)
{
    char message[MOTOR_COMMAND_REPLY_SIZE];
    send_reply = send;
    length = discard = fault_latched = moving = 0U;
    last_byte_tick = last_command_tick = 0U;
    stop();
    (void)snprintf(message, sizeof(message),
        "READY UNIT=PERCENT SCALE1=%.0f SCALE2=%.0f CPS KP=%.3f KI=%.3f KD=%.3f PWM_MAX=%.0f\r\n",
        (double)Motor_FullScaleCountsPerSecond[0], (double)Motor_FullScaleCountsPerSecond[1],
        (double)Motor_MeasuredCountsPID.kp, (double)Motor_MeasuredCountsPID.ki,
        (double)Motor_MeasuredCountsPID.kd, (double)Motor_MeasuredCountsPID.output_max);
    reply(message);
}

static void execute(uint32_t tick)
{
    unsigned int first = digits[0] * 10U + digits[1];
    unsigned int second = digits[2] * 10U + digits[3];
    float first_percent = (float)first * 100.0f / 99.0f;
    float second_percent = (float)second * 100.0f / 99.0f;
    char message[MOTOR_COMMAND_REPLY_SIZE];
    length = 0U;
    if (first != 0U || second != 0U)
    {
        if (fault_latched)
        {
            Motor_CommandFault("LATCHED_SEND_0000");
            return;
        }
    }
    if (CSGO(first_percent, second_percent) != HAL_OK)
    {
        Motor_CommandFault("CONTROL");
        return;
    }
    fault_latched = 0U;
    moving = (uint8_t)(first != 0U || second != 0U);
    last_command_tick = tick;
    (void)snprintf(message, sizeof(message),
        "ACK RAW=%02u%02u M1=%.2f M2=%.2f UNIT=PERCENT CPS1=%.1f CPS2=%.1f PWM_MAX=%.0f\r\n",
        first, second, (double)first_percent, (double)second_percent,
        (double)(first_percent * Motor_FullScaleCountsPerSecond[0] / 100.0f),
        (double)(second_percent * Motor_FullScaleCountsPerSecond[1] / 100.0f),
        (double)Motor_MeasuredCountsPID.output_max);
    reply(message);
}

void Motor_CommandPoll(uint32_t tick)
{
    uint32_t elapsed = tick - last_byte_tick;
    if (moving && (uint32_t)(tick - last_command_tick) >= MOTOR_COMMAND_LEASE_MS)
    {
        Motor_CommandFault("LINK_TIMEOUT");
    }
    if (discard)
    {
        if (elapsed >= MOTOR_COMMAND_IDLE_MS) discard = 0U;
        return;
    }
    if (length == 4U && elapsed >= MOTOR_COMMAND_IDLE_MS)
        execute(tick);
    else if (length != 0U && elapsed >= MOTOR_COMMAND_PARTIAL_MS)
        Motor_CommandFault("PARTIAL_TIMEOUT");
}

void Motor_CommandReceive(uint8_t byte, uint32_t tick)
{
    Motor_CommandPoll(tick);
    last_byte_tick = tick;
    if (byte == '\r' || byte == '\n')
    {
        if (!discard && length == 4U) execute(tick);
        else if (!discard && length != 0U) Motor_CommandFault("LENGTH");
        length = discard = 0U;
        return;
    }
    if (discard) return;
    if (byte < '0' || byte > '9' || length >= 4U)
    {
        Motor_CommandFault("FRAME");
        return;
    }
    digits[length++] = (uint8_t)(byte - '0');
}
