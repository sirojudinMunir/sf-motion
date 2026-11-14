/**
 * vbus_prot.h - DC bus under-voltage / over-voltage protection for FOC drives
 *
 * Features:
 *  - Separate trip and clear thresholds (hysteresis) for UV and OV.
 *  - Time debounce on trip (OV is usually short or zero, UV a bit longer to
 *    ignore dips caused by current spikes) and on clear.
 *
 * Use the SAME unit for all voltages (volts).
 *
 */

#ifndef VBUS_PROT_H_
#define VBUS_PROT_H_

#include <stdint.h>

#define VBUS_FAULT_NONE   0x00u
#define VBUS_FAULT_UV     0x01u
#define VBUS_FAULT_OV     0x02u

#define VBUS_UV_HYSTERESIS 1.0f
#define VBUS_OV_HYSTERESIS 1.0f

typedef struct
{
  /* ---- configuration ---- */
  float uv_trip;       /* Trip when filtered Vbus < this, V            */
  float uv_clear;      /* Clear when filtered Vbus > this, V           */
  float ov_trip;       /* Trip when raw Vbus > this, V                 */
  float ov_clear;      /* Clear when raw Vbus < this, V                */
  float uv_trip_time;  /* UV must persist this long to trip, s         */
  float ov_trip_time;  /* OV must persist this long to trip, s (0=fast)*/
  float clear_time;    /* Healthy for this long before clearing, s     */
  float filt_alpha;    /* Low-pass coefficient 0..1 (1 = no filter)    */
  float dt;            /* Update period, s                             */

  /* ---- state ---- */
  float v;
  float uv_timer;
  float ov_timer;
  float uv_ok_timer;
  float ov_ok_timer;
  uint8_t fault;
  uint8_t last_fault;
} vbus_t;


int vbus_init(vbus_t *hvbus, float uv, float ov, float dt);
_Bool vbus_update(vbus_t *hvbus, float vbus_raw);
uint8_t vbus_get_fault(vbus_t *hvbus);
void vbus_reset(vbus_t *hvbus);

#endif /* VBUS_PROT_H_ */
