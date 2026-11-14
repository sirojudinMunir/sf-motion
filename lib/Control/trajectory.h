#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

/*
 * S-curve trajectory generator 7-segmen (jerk-limited).
 */

typedef struct {
    float start_pos;
    float target_pos;
    float v_max;
    float a_max;
    float j_max;

    float direction;
    float distance;

    float Tj1;
    float Ta;
    float Tv;
    float Td;
    float alim;
    float vlim;

    float t1, t2, t3, t4, t5, t6, t7;

    float p1, v1;
    float p2, v2;
    float p3, v3;
    float p4;
    float p5, v5;
    float p6, v6;

    float time;
    float pos;
    float velocity;
    float acceleration;
    uint8_t finished;
} s_curve_t;

int8_t s_curve_start(s_curve_t *sc, float start_pos, float target_pos);
void s_curve_reset(s_curve_t *sc, float initial_pos);
void s_curve_set_v_max(s_curve_t *sc, float v_max);
void s_curve_set_a_max(s_curve_t *sc, float a_max);
void s_curve_set_j_max(s_curve_t *sc, float j_max);

int8_t s_curve_update(s_curve_t *sc, float dt);

float s_curve_get_pos(s_curve_t *sc);
float s_curve_get_vel(s_curve_t *sc);
float s_curve_get_accel(s_curve_t *sc);
float s_curve_get_duration(const s_curve_t *sc);

#endif /* TRAJECTORY_H */