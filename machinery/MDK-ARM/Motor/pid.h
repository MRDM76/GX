#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float output_min, output_max;
    float integral_min, integral_max;
    float derivative_tau; /* derivative low-pass time constant, seconds */
} PID_Config;

typedef struct {
    PID_Config config;
    float integral;
    float previous_measurement;
    float derivative;
    float output;
    unsigned char initialized;
    unsigned char has_previous;
} PID_Controller;

/* No HAL dependency. Return 1 on success, 0 on invalid config/input/math.
 * Gains >=0. dt is seconds; Ki/Kd already account for dt in PID_Update.
 * Derivative is on measurement (no setpoint derivative kick).
 * Conditional integration prevents further windup at output saturation.
 */
int PID_Init(PID_Controller *pid, const PID_Config *config);
void PID_Reset(PID_Controller *pid);
int PID_Update(PID_Controller *pid, float target, float measurement,
               float dt_seconds, float *output);

#ifdef __cplusplus
}
#endif
#endif
