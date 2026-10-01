#include "motor.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    Encoder_State s;
    Encoder_Reset(&s, 65530U);
    assert(Encoder_Sample(&s, 4U, 10U));
    assert(s.delta == 10 && s.position == 10 && s.counts_per_second == 1000.0f);
    assert(s.rpm == 0.0f && s.counts_per_turn == 0);
    s.counts_per_turn = 1000;
    assert(Encoder_Sample(&s, 65530U, 20U));
    assert(s.delta == -10 && s.position == 0 && s.rpm == -30.0f);
    s.sign = -1;
    assert(Encoder_Sample(&s, 65520U, 10U));
    assert(s.delta == 10 && s.position == 10);
    assert(!Encoder_Sample(&s, 40U, 0U) && s.previous == 65520U);
    assert(!Encoder_Sample(&s, 500U, 101U));
    assert(!s.valid && s.delta == 0 && s.rpm == 0.0f && s.position == 10);
    assert(Encoder_Sample(&s, 500U, 10U) && s.delta == 0);
    assert(!Encoder_Sample(&s, (uint16_t)(500U + 32768U), 10U));
    Encoder_Reset(&s, 0);
    s.position = INT64_MAX;
    assert(!Encoder_Sample(&s, 1, 10));
    assert(s.position == INT64_MAX);
    puts("PASS: encoder forward/reverse wrap, variable dt, sign, CPR, zero interval, stale and ambiguous samples, overflow");
    return 0;
}
