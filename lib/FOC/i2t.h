/**
 * i2t.h - i2t (thermal overload) motor protection for FOC drives
 *
 * Model:  acc += (I^2 - I_rated^2) * dt        (acc clamped to >= 0)
 *         trip when acc >= (I_max^2 - I_rated^2) * t_max
 *
 *  - I = sqrt(Id^2 + Iq^2) in the dq frame (no sqrt needed, we use I^2).
 *  - Below rated current the accumulator cools down (cool_gain scales this).
 *  - After a trip the fault stays latched until the accumulator has cooled
 *    below reset_ratio (hysteresis), then it clears automatically.
 *
 * All currents must use the SAME unit and scaling (e.g. amps, peak value of
 * the phase current, as given by the amplitude-invariant Clarke/Park).
 *
 */

#ifndef I2T_H_
#define I2T_H_

#include <stdint.h>

typedef struct
{
    /* ---- configuration (set by i2t_Init, may be tuned afterwards) ---- */
    float i_rated;       /* Continuous (nominal) current, A              */
    float i_max;         /* Overload current, A                          */
    float t_max;         /* Time allowed at i_max before trip, s         */
    float dt;            /* Update period, s (1 / call frequency)        */
    float cool_gain;     /* Cooling speed multiplier (1.0 = symmetric)   */
    float warn_ratio;    /* Load ratio where derating begins (0..1)      */
    float reset_ratio;   /* Load ratio below which a trip is cleared     */

    /* ---- derived ---- */
    float i2_rated;      /* i_rated^2                                    */
    float acc_limit;     /* Trip threshold of the accumulator            */

    /* ---- state ---- */
    float acc;           /* Thermal accumulator, A^2*s                   */
    _Bool tripped;       /* Latched overload flag                        */
} i2t_t;

int i2t_init(i2t_t *hi2t, float dt);
int i2t_set_config(i2t_t *hi2t, float i_rated, float i_max, float t_max);
void i2t_update(i2t_t *hi2t, float id, float iq);
float i2t_get_load(const i2t_t *hi2t);
float i2t_get_current_limit(const i2t_t *hi2t);
_Bool i2t_is_tripped(const i2t_t *hi2t);
void i2t_reset(i2t_t *hi2t);

#endif /* I2T_H_ */