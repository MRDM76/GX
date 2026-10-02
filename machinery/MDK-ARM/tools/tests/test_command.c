#include "motor_command.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float target_first, target_second;
static unsigned int apply_count;
static int fail_apply, fail_send;
static char response[MOTOR_COMMAND_REPLY_SIZE];

HAL_StatusTypeDef CSGO(float first, float second)
{
    if (fail_apply && (first != 0 || second != 0)) return HAL_ERROR;
    target_first = first;
    target_second = second;
    ++apply_count;
    return HAL_OK;
}

static int capture(const char *message)
{
    if (fail_send) return 0;
    assert(strlen(message) < sizeof(response));
    memcpy(response, message, strlen(message) + 1U);
    return 1;
}

static void feed(const char *bytes, uint32_t tick)
{
    while (*bytes) Motor_CommandReceive((uint8_t)*bytes++, tick);
}

int main(void)
{
    unsigned int before;
    Motor_CommandInit(capture);
    assert(strstr(response, "READY UNIT=PERCENT") != NULL);
    feed("3060", 1);
    assert(target_first == 0 && target_second == 0);
    Motor_CommandPoll(20);
    assert(target_first == 0);
    Motor_CommandPoll(21);
    assert(fabsf(target_first - 30.30303f) < 0.0001f);
    assert(fabsf(target_second - 60.60606f) < 0.0001f);
    assert(strstr(response, "ACK RAW=3060") != NULL);
    feed("0000\r\n", 30);
    assert(target_first == 0 && target_second == 0);
    feed("0099\n", 40);
    assert(target_second == 100 && target_first == 0);
    Motor_CommandPoll(539);
    assert(target_second == 100);
    Motor_CommandPoll(540);
    assert(target_second == 0 && strstr(response, "LINK_TIMEOUT") != NULL);
    feed("0099\n", 550);
    assert(target_second == 0 && strstr(response, "LATCHED") != NULL);
    feed("0000\n3060\n", 560);
    assert(target_second > 60);
    feed("99999\n", 570);
    assert(target_first == 0 && strstr(response, "FRAME") != NULL);
    feed("0000\n", 580);
    before = apply_count;
    feed("12", 590);
    Motor_CommandPoll(689);
    assert(apply_count == before);
    Motor_CommandPoll(690);
    assert(strstr(response, "PARTIAL_TIMEOUT") != NULL);
    feed("\n0000\n", 700);
    feed("1x34\n", 710);
    assert(target_first == 0 && strstr(response, "FRAME") != NULL);
    feed("0000\n", 720);
    feed("123\n", 730);
    assert(strstr(response, "LENGTH") != NULL);
    feed("0000\n", 740);
    feed("12", 750);
    feed("34\n", 799);
    assert(target_first > 12 && target_second > 34);
    Motor_CommandInit(capture);
    feed("9900\n", UINT32_MAX - 100U);
    Motor_CommandPoll(398);
    assert(target_first == 100);
    Motor_CommandPoll(399);
    assert(target_first == 0 && strstr(response, "LINK_TIMEOUT") != NULL);
    Motor_CommandInit(capture);
    fail_apply = 1;
    feed("0101\n", 1);
    assert(strstr(response, "CONTROL") != NULL && target_first == 0);
    fail_apply = 0;
    feed("0000\n", 2);
    fail_send = 1;
    feed("9999\n", 3);
    assert(target_first == 0 && target_second == 0);
    fail_send = 0;
    feed("9999\n", 4);
    assert(strstr(response, "LATCHED") != NULL);
    Motor_CommandInit(capture);
    feed("30600000\n", 1);
    assert(target_first == 0 && strstr(response, "FRAME") != NULL);
    puts("PASS: four digits, percent mapping, CRLF/raw idle, split/long/bad frames, lease, fault latch, wrap, send failure");
    return 0;
}
