#ifndef PVT_H_
#define PVT_H_

#include <stdint.h>

#define MAX_PVT_POINTS  256

typedef struct {
  float position;
  float velocity;
} pvt_point_t;

typedef struct {
  pvt_point_t points[MAX_PVT_POINTS];
  pvt_point_t current_point;
  _Bool run;
  uint32_t tick;
  uint32_t tick_prescaller;
  uint32_t available_points;
} pvt_t;

void pvt_init(pvt_t *hpvt, uint32_t prescaller);
void pvt_update(pvt_t *hpvt);
int pvt_fill_buffer(pvt_t *hpvt, pvt_point_t point);
void pvt_run(pvt_t *hpvt);
void pvt_stop(pvt_t *hpvt);
pvt_point_t pvt_get_current_point(pvt_t *hpvt);

#endif /* PVT_H_ */