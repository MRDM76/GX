#include "pid.h"
#include <float.h>

static int finite_value(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

static float limit(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->has_previous = 0U;
}

int PID_Init(PID_Controller *pid, const PID_Config *config)
{
    if (pid == 0 || config == 0 ||
        !finite_value(config->kp) || !finite_value(config->ki) ||
        !finite_value(config->kd) || !finite_value(config->derivative_tau) ||
        !finite_value(config->output_min) || !finite_value(config->output_max) ||
        !finite_value(config->integral_min) || !finite_value(config->integral_max) ||
        config->kp < 0.0f || config->ki < 0.0f || config->kd < 0.0f ||
        config->derivative_tau < 0.0f || config->output_min >= config->output_max ||
        config->integral_min > 0.0f || config->integral_max < 0.0f)
        return 0;
    pid->config = *config;
    pid->initialized = 1U;
    PID_Reset(pid);
    return 1;
}

int PID_Update(PID_Controller *pid, float target, float measurement,
               float dt_seconds, float *output)
{
    float error, p, delta_i, candidate_i, derivative, raw_derivative;
    float tau_dt, sum;
    if (output == 0) return 0;
    *output = 0.0f;
    if (pid == 0 || !pid->initialized || !finite_value(target) ||
        !finite_value(measurement) || !finite_value(dt_seconds) || dt_seconds <= 0.0f)
        return 0;
    error = target - measurement;
    p = pid->config.kp * error;
    delta_i = pid->config.ki * error * dt_seconds;
    candidate_i = pid->integral + delta_i;
    derivative = 0.0f;
    if (!finite_value(error) || !finite_value(p) || !finite_value(delta_i) ||
        !finite_value(candidate_i)) return 0;
    candidate_i = limit(candidate_i, pid->config.integral_min, pid->config.integral_max);
    if (pid->has_previous && pid->config.kd > 0.0f)
    {
        raw_derivative = -(measurement - pid->previous_measurement) / dt_seconds;
        tau_dt = pid->config.derivative_tau + dt_seconds;
        if (!finite_value(raw_derivative) || !finite_value(tau_dt)) return 0;
        derivative = pid->derivative + (dt_seconds / tau_dt) * (raw_derivative - pid->derivative);
    }
    sum = p + candidate_i + pid->config.kd * derivative;
    if (!finite_value(derivative) || !finite_value(sum)) return 0;
    if ((sum > pid->config.output_max && candidate_i > pid->integral) ||
        (sum < pid->config.output_min && candidate_i < pid->integral))
    {
        candidate_i = pid->integral;
        sum = p + candidate_i + pid->config.kd * derivative;
        if (!finite_value(sum)) return 0;
    }
    pid->integral = candidate_i;
    pid->derivative = derivative;
    pid->previous_measurement = measurement;
    pid->has_previous = 1U;
    pid->output = limit(sum, pid->config.output_min, pid->config.output_max);
    *output = pid->output;
    return 1;
}
