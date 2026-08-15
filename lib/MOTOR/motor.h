#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <math.h>
#include "FOC_math.h"
#include "lpf.h"

typedef struct {
  volatile uint32_t *p_pwm_va;
  volatile uint32_t *p_pwm_vb;
  volatile uint32_t *p_pwm_vc;
  uint32_t *p_adc_ia;
  uint32_t *p_adc_ib;
  uint32_t *p_adc_ic;
  uint32_t *p_adc_vbus;
  float current_sense_gain;
  float ia_offset;
  float ib_offset;
  float ic_offset;
  float vbus_sense_gain;
  float vbus_actual;
  float vbus_actual_filtered;
  float ia;
  float ib;
  float ic;
  float elec_angle_rad;
  SecondOrderLPF ia_lpf;
  SecondOrderLPF ib_lpf;
  SecondOrderLPF ic_lpf;
  SecondOrderLPF vbus_lpf;
}motor_t;

void motor_init_pwm(motor_t *hmotor, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c);
void motor_init_adc_current_sense(motor_t *hmotor, uint32_t *adc_a, uint32_t *adc_b, uint32_t *adc_c);
void motor_init_adc_power_voltage_sense(motor_t *hmotor, uint32_t *adc_v);
void motor_init_lpf_current_sense(motor_t *hmotor, float cutoff_freq, float sampling_freq);
void motor_init_lpf_power_voltage_sense(motor_t *hmotor, float cutoff_freq, float sampling_freq);
void motor_set_current_sense_gain(motor_t *hmotor, float gain);
void motor_set_power_voltage_sense_gain(motor_t *hmotor, float gain);
void motor_set_current_sense_offset(motor_t *hmotor, float ia_offset, float ib_offset, float ic_offset);
void motor_set_pwm(motor_t *hmotor, uint32_t pwm_a, uint32_t pwm_b, uint32_t pwm_c);
void motor_get_current(motor_t *hmotor, float *ia, float *ib, float *ic);
void motor_get_power_voltage(motor_t *hmotor, float *pv);
void motor_calculate_current(motor_t *hmotor);
void motor_calculate_power_voltage(motor_t *hmotor);


#endif // MOTOR_H