#include "vbus_protection.h"

#define VBUS_TWO_PI  6.28318530718f

int vbus_init(vbus_t *hvbus, float uv, float ov, float dt) {
  if (hvbus == 0 || dt <= 0.0f) {
    return -1;
  }

  hvbus->uv_trip = uv;
  hvbus->uv_clear = uv + VBUS_UV_HYSTERESIS;
  hvbus->ov_clear = ov - VBUS_OV_HYSTERESIS;
  hvbus->ov_trip = ov;
  hvbus->dt = dt;
  hvbus->filt_alpha = 1.0f;

  hvbus->uv_trip_time = 0.005f;   /* 5 ms  */
  hvbus->ov_trip_time = 0.0f;     /* immediate */
  hvbus->clear_time = 0.050f;     /* 50 ms */

  vbus_reset(hvbus);

  return 0;
}

_Bool vbus_update(vbus_t *hvbus, float vbus_raw) {
  if (hvbus == 0) {
    return 0;
  }

  hvbus->v += hvbus->filt_alpha * (vbus_raw - hvbus->v);

  if (!(hvbus->fault & VBUS_FAULT_UV)) {
    if (hvbus->v < hvbus->uv_trip) {
      hvbus->uv_timer += hvbus->dt;
      if (hvbus->uv_timer >= hvbus->uv_trip_time) {
          hvbus->fault |= VBUS_FAULT_UV;
          hvbus->uv_ok_timer = 0.0f;
      }
    }
    else {
      hvbus->uv_timer = 0.0f;
    }
  }
  else {
    if (hvbus->v > hvbus->uv_clear) {
      hvbus->uv_ok_timer += hvbus->dt;
      if (hvbus->uv_ok_timer >= hvbus->clear_time) {
        hvbus->fault &= (uint8_t)~VBUS_FAULT_UV;
        hvbus->uv_timer = 0.0f;
      }
    }
    else {
      hvbus->uv_ok_timer = 0.0f;
    }
  }

  if (!(hvbus->fault & VBUS_FAULT_OV)) {
    if (vbus_raw > hvbus->ov_trip) {
      hvbus->ov_timer += hvbus->dt;
      if (hvbus->ov_timer >= hvbus->ov_trip_time) {
          hvbus->fault |= VBUS_FAULT_OV;
          hvbus->ov_ok_timer = 0.0f;
      }
    }
    else {
      hvbus->ov_timer = 0.0f;
    }
  }
  else {
    if (vbus_raw < hvbus->ov_clear) {
      hvbus->ov_ok_timer += hvbus->dt;
      if (hvbus->ov_ok_timer >= hvbus->clear_time) {
        hvbus->fault &= (uint8_t)~VBUS_FAULT_OV;
        hvbus->ov_timer = 0.0f;
      }
    }
    else {
      hvbus->ov_ok_timer = 0.0f;
    }
  }

  if (hvbus->last_fault != hvbus->fault) {
    hvbus->last_fault = hvbus->fault;
    return 1;
  }
  return 0;
}

uint8_t vbus_get_fault(vbus_t *hvbus) {
  return hvbus->fault;
}

void vbus_reset(vbus_t *hvbus) {
  hvbus->v = 0.0f;
  hvbus->fault = VBUS_FAULT_NONE;
  hvbus->last_fault = VBUS_FAULT_NONE;
  hvbus->uv_timer = 0.0f;
  hvbus->ov_timer = 0.0f;
  hvbus->uv_ok_timer = 0.0f;
  hvbus->ov_ok_timer = 0.0f;
}
