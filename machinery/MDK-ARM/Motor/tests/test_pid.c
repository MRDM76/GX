#include "pid.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static void near_value(float actual, float expected, float tolerance)
{ assert(fabsf(actual - expected) < tolerance); }

int main(void)
{
    PID_Controller p = {0};
    PID_Config cfg = {2, 1, 0, 0, 100, 0, 100, 0};
    float out, old_integral;
    int i;
    PID_Config rpm_config;
    assert(!PID_MakeRPMConfig(0, &rpm_config));
    assert(!PID_MakeRPMConfig(600, NULL));
    assert(PID_MakeRPMConfig(600, &rpm_config));
    near_value(rpm_config.kp, 0.5f, 0.0001f);
    near_value(rpm_config.ki, 4.0f, 0.0001f);
    assert(rpm_config.output_max == 300.0f);
    assert(!PID_Update(&p, 1, 0, 0.01f, &out));
    assert(PID_Init(&p, &cfg));
    assert(PID_Update(&p, 10, 0, 0.1f, &out));
    near_value(out, 21, 0.0001f);
    assert(PID_Update(&p, 10, 0, 0.2f, &out));
    near_value(out, 23, 0.0001f);
    PID_Reset(&p);
    for (i = 0; i < 1000; ++i) assert(PID_Update(&p, 1000, 0, 0.01f, &out));
    near_value(out, 100, 0.0001f);
    near_value(p.integral, 0, 0.0001f);
    assert(PID_Update(&p, 0, 0, 0.01f, &out));
    near_value(out, 0, 0.0001f);
    cfg.kp = 0; cfg.ki = 10; cfg.integral_max = 5;
    assert(PID_Init(&p, &cfg));
    assert(PID_Update(&p, 10, 0, 1, &out));
    near_value(out, 5, 0.0001f);
    cfg.kd = 1; cfg.ki = 0; cfg.output_min = -100;
    cfg.derivative_tau = 0.1f;
    assert(PID_Init(&p, &cfg));
    assert(PID_Update(&p, 0, 10, 0.1f, &out));
    near_value(out, 0, 0.0001f);
    assert(PID_Update(&p, 100, 10, 0.1f, &out));
    near_value(out, 0, 0.0001f); /* target change has no derivative kick */
    assert(PID_Update(&p, 100, 12, 0.1f, &out));
    near_value(out, -10, 0.0001f); /* filtered derivative of measurement */
    old_integral = p.integral;
    assert(!PID_Update(&p, NAN, 0, 0.01f, &out));
    assert(!PID_Update(&p, 0, INFINITY, 0.01f, &out));
    assert(!PID_Update(&p, 0, 0, 0, &out));
    assert(!PID_Update(&p, FLT_MAX, -FLT_MAX, 0.01f, &out));
    assert(p.integral == old_integral);
    cfg.kp = -1;
    assert(!PID_Init(&p, &cfg));

    /* Numerical plant only, not a model calibrated to the actual N20. */
    cfg = (PID_Config){2, 15, 0, 0, 1000, 0, 1000, 0};
    assert(PID_Init(&p, &cfg));
    {
        float speed = 0, load;
        for (i = 0; i < 3000; ++i)
        {
            load = i < 1500 ? 0.0f : 20.0f;
            assert(PID_Update(&p, 100, speed, 0.01f, &out));
            assert(out >= 0 && out <= 1000);
            speed += 0.01f * (0.2f * out - load - speed) / 0.15f;
            if (i == 1499) near_value(speed, 100, 0.2f);
        }
        near_value(speed, 100, 0.2f);
    }
    puts("PASS: PID P/I/dt, windup recovery, integral limits, derivative filter/no kick, invalid inputs, simulated speed/load tracking");
    return 0;
}
