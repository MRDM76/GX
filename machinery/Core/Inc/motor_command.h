#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdint.h>

#define MOTOR_COMMAND_IDLE_MS 20U
#define MOTOR_COMMAND_PARTIAL_MS 100U
#define MOTOR_COMMAND_LEASE_MS 500U
#define MOTOR_COMMAND_REPLY_SIZE 192U

typedef int (*Motor_CommandSend)(const char *message);

void Motor_CommandInit(Motor_CommandSend send);
void Motor_CommandReceive(uint8_t byte, uint32_t tick);
void Motor_CommandPoll(uint32_t tick);
void Motor_CommandFault(const char *reason);

#endif
