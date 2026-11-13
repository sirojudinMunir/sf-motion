#include "i2t.h"

int i2t_init(i2t_t *hi2t, float dt) {
  if (hi2t == 0 || dt <= 0.0f) {
    return -1;
  }

  hi2t->dt = dt;
  hi2t->cool_gain = 1.0f;
  hi2t->warn_ratio = 0.8f;
  hi2t->reset_ratio = 0.5f;

  i2t_reset(hi2t);
  return 0;
}

int i2t_set_config(i2t_t *hi2t, float i_rated, float i_max, float t_max) {
  if (i_rated <= 0 || i_max <= 0 || t_max <= 0) {
    return -1;
  }

  hi2t->i_rated = i_rated;
  hi2t->i_max = i_max;
  hi2t->t_max = t_max;

  hi2t->i2_rated = i_rated * i_rated;
  hi2t->acc_limit = (i_max * i_max - hi2t->i2_rated) * t_max;

  i2t_reset(hi2t);
  return 0;
}

void i2t_update(i2t_t *hi2t, float id, float iq) {
  float i_squared = id * id + iq * iq;
  float delta = (i_squared - hi2t->i2_rated) * hi2t->dt;

  if (delta < 0.0f) {
    delta *= hi2t->cool_gain;      /* cooling phase */
  }

  hi2t->acc += delta;

  if (hi2t->acc < 0.0f) {
    hi2t->acc = 0.0f;              /* motor cannot be "colder than cold" */
  }

  if (hi2t->acc >= hi2t->acc_limit) {
    hi2t->acc = hi2t->acc_limit;  /* clamp so cool-down starts at full */
    hi2t->tripped = 1;
  }
  else if (hi2t->tripped && hi2t->acc <= hi2t->reset_ratio * hi2t->acc_limit) {
    hi2t->tripped = 0;         /* cooled enough, auto-clear */
  }
}

float i2t_get_load(const i2t_t *hi2t) {
  return hi2t->acc / hi2t->acc_limit;
}

float i2t_get_current_limit(const i2t_t *hi2t) {
  float load = i2t_get_load(hi2t);

  if (hi2t->tripped) {
    return 0.0f;
  }

  if (load <= hi2t->warn_ratio) {
    return hi2t->i_max;
  }

  /* linear ramp i_max -> i_rated between warn_ratio and 1.0 */
  float k = (load - hi2t->warn_ratio) / (1.0f - hi2t->warn_ratio);
  if (k > 1.0f) {
    k = 1.0f;
  }

  return hi2t->i_max - k * (hi2t->i_max - hi2t->i_rated);
}

_Bool i2t_is_tripped(const i2t_t *hi2t) {
  return hi2t->tripped;
}

void i2t_reset(i2t_t *hi2t) {
  hi2t->acc = 0.0f;
  hi2t->tripped = 0;
}