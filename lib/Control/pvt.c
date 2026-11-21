#include "pvt.h"

void pvt_init(pvt_t *hpvt, uint32_t prescaller) {
  hpvt->available_points = MAX_PVT_POINTS;
  hpvt->tick_prescaller = prescaller;
}

void pvt_update(pvt_t *hpvt) {
  if (!hpvt->run) return;
  if (++hpvt->tick >= hpvt->tick_prescaller) {
    hpvt->tick = 0;
    if (hpvt->available_points == MAX_PVT_POINTS) {
      hpvt->run = 0;
      return;
    }
    hpvt->current_point = hpvt->points[0];
    hpvt->available_points++;
    uint32_t n = MAX_PVT_POINTS - hpvt->available_points;
    for (uint32_t i = 0; i < n; i++) {
      hpvt->points[i] = hpvt->points[i+1];
    }
  }
}

int pvt_fill_buffer(pvt_t *hpvt, pvt_point_t point) {
  if (hpvt->available_points == 0) return -1;
  uint32_t idx = MAX_PVT_POINTS - hpvt->available_points;
  hpvt->points[idx] = point;
  hpvt->available_points--;
  return 0;
}

void pvt_run(pvt_t *hpvt) {
  if (hpvt->available_points == MAX_PVT_POINTS) {
    hpvt->run = 0;
    return;
  }

  // Load the first queued point immediately so motion never starts from the
  // zero-initialized current_point.
  hpvt->current_point = hpvt->points[0];
  hpvt->available_points++;
  uint32_t remaining = MAX_PVT_POINTS - hpvt->available_points;
  for (uint32_t i = 0; i < remaining; i++) {
    hpvt->points[i] = hpvt->points[i + 1];
  }
  hpvt->tick = 0;
  hpvt->run = 1;
}

void pvt_stop(pvt_t *hpvt) {
  hpvt->run = 0;
}

pvt_point_t pvt_get_current_point(pvt_t *hpvt) {
  return hpvt->current_point;
}
