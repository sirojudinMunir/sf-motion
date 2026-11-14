#include "trajectory.h"
#include <math.h>

static float f_abs(float x) { return x < 0.0f ? -x : x; }

void s_curve_reset(s_curve_t *sc, float initial_pos) {
    if (!sc) return;

    sc->start_pos = initial_pos;
    sc->target_pos = initial_pos;
    sc->direction = 1.0f;
    sc->distance = 0.0f;
    sc->time = 0.0f;
    sc->pos = initial_pos;
    sc->velocity = 0.0f;
    sc->acceleration = 0.0f;
    sc->finished = 1;
}

int8_t s_curve_start(s_curve_t *sc, float start_pos, float target_pos) {
  float v_max = sc->v_max;
  float a_max = sc->a_max;
  float j_max = sc->j_max;
  if (!sc || v_max <= 0.0f || a_max <= 0.0f || j_max <= 0.0f) {
      return -1;
  }

    sc->start_pos = start_pos;
    sc->target_pos = target_pos;

    float h = target_pos - start_pos;
    sc->direction = (h < 0.0f) ? -1.0f : 1.0f;
    sc->distance  = f_abs(h);

    sc->time = 0.0f;
    sc->pos = start_pos;
    sc->velocity = 0.0f;
    sc->acceleration = 0.0f;
    sc->finished = 0;

    /* zero distance */
    if (sc->distance < 1e-9f) {
        sc->Tj1 = sc->Ta = sc->Tv = sc->Td = 0.0f;
        sc->alim = sc->vlim = 0.0f;
        sc->t1 = sc->t2 = sc->t3 = sc->t4 = sc->t5 = sc->t6 = sc->t7 = 0.0f;
        sc->p1 = sc->v1 = sc->p2 = sc->v2 = sc->p3 = sc->v3 = sc->p4 = 0.0f;
        sc->p5 = sc->v5 = sc->p6 = sc->v6 = 0.0f;
        sc->finished = 1;
        return 0;
    }

    float h_d = sc->distance;

    /* ---- Langkah 1: asumsikan v_max tercapai, hitung fase percepatan ---- */
    float Tj1, Ta, alim;
    if ((a_max * a_max) / j_max <= v_max) {
        /* a_max tercapai sebelum kecepatan mencapai v_max */
        Tj1  = a_max / j_max;
        Ta   = Tj1 + v_max / a_max;
        alim = a_max;
    } else {
        /* v_max tercapai duluan -> percepatan hanya membentuk segitiga */
        Tj1  = sqrtf(v_max / j_max);
        Ta   = 2.0f * Tj1;
        alim = j_max * Tj1;
    }

    float dist_accel_decel = v_max * Ta; /* jarak fase percepatan + perlambatan */

    float vlim, Tv, Td;
    if (h_d >= dist_accel_decel) {
        /* v_max benar-benar tercapai -> ada fase kecepatan konstan */
        vlim = v_max;
        Tv   = h_d / v_max - Ta;
        Td   = Ta;
    } else {
        /* Jarak terlalu pendek untuk mencapai v_max -> hitung ulang */
        Tv = 0.0f;
        if (h_d >= 2.0f * (a_max * a_max * a_max) / (j_max * j_max)) {
            /* a_max tetap tercapai, tapi tanpa fase kecepatan konstan */
            Tj1 = a_max / j_max;
            Ta  = Tj1 * 0.5f + sqrtf((Tj1 * 0.5f) * (Tj1 * 0.5f) + h_d / a_max);
            alim = a_max;
        } else {
            /* a_max juga tidak tercapai -> profil jerk murni (segitiga ganda) */
            Ta   = cbrtf(4.0f * h_d / j_max);
            Tj1  = Ta * 0.5f;
            alim = j_max * Tj1;
        }
        Td   = Ta;
        vlim = alim * (Ta - Tj1);
    }

    sc->Tj1 = Tj1;
    sc->Ta  = Ta;
    sc->Tv  = Tv;
    sc->Td  = Td;
    sc->alim = alim;
    sc->vlim = vlim;

    float j = j_max;
    float a = alim;

    /* --- akhir fase 1 (jerk+) --- */
    float t1 = Tj1;
    float v1 = 0.5f * j * t1 * t1;
    float p1 = (1.0f / 6.0f) * j * t1 * t1 * t1;

    /* --- akhir fase 2 (percepatan konstan) --- */
    float dt2 = Ta - 2.0f * Tj1;
    if (dt2 < 0.0f) dt2 = 0.0f;
    float v2 = v1 + a * dt2;
    float p2 = p1 + v1 * dt2 + 0.5f * a * dt2 * dt2;

    /* --- akhir fase 3 (jerk-, kecepatan capai vlim) --- */
    float t3d = Tj1;
    float v3 = v2 + a * t3d - 0.5f * j * t3d * t3d; /* == vlim */
    float p3 = p2 + v2 * t3d + 0.5f * a * t3d * t3d
               - (1.0f / 6.0f) * j * t3d * t3d * t3d;

    /* --- akhir fase 4 (kecepatan konstan) --- */
    float p4 = p3 + vlim * Tv;

    /* --- akhir fase 5 (jerk-, perlambatan mulai) --- */
    float t5d = Tj1;
    float v5 = vlim - 0.5f * j * t5d * t5d;
    float p5 = p4 + vlim * t5d - (1.0f / 6.0f) * j * t5d * t5d * t5d;

    /* --- akhir fase 6 (perlambatan konstan) --- */
    float dt6 = Td - 2.0f * Tj1;
    if (dt6 < 0.0f) dt6 = 0.0f;
    float v6 = v5 - a * dt6;
    float p6 = p5 + v5 * dt6 - 0.5f * a * dt6 * dt6;

    sc->t1 = t1;         sc->v1 = v1; sc->p1 = p1;
    sc->t2 = t1 + dt2;   sc->v2 = v2; sc->p2 = p2;
    sc->t3 = sc->t2 + t3d; sc->v3 = v3; sc->p3 = p3;
    sc->t4 = sc->t3 + Tv;  sc->p4 = p4;
    sc->t5 = sc->t4 + t5d; sc->v5 = v5; sc->p5 = p5;
    sc->t6 = sc->t5 + dt6; sc->v6 = v6; sc->p6 = p6;
    sc->t7 = sc->t6 + Tj1;

    return 0;
}

int8_t s_curve_update(s_curve_t *sc, float dt) {
    if (!sc) return -1;
    if (sc->finished) return -1;

    sc->time += dt;

    float j = sc->j_max;
    float a = sc->alim;
    float t = sc->time;

    float pos, vel, accel;

    if (t >= sc->t7) {
        pos = sc->distance;
        vel = 0.0f;
        accel = 0.0f;
        sc->finished = 1;
    } else if (t < sc->t1) {
        float tau = t;
        accel = j * tau;
        vel   = 0.5f * j * tau * tau;
        pos   = (1.0f / 6.0f) * j * tau * tau * tau;
    } else if (t < sc->t2) {
        float tau = t - sc->t1;
        accel = a;
        vel   = sc->v1 + a * tau;
        pos   = sc->p1 + sc->v1 * tau + 0.5f * a * tau * tau;
    } else if (t < sc->t3) {
        float tau = t - sc->t2;
        accel = a - j * tau;
        vel   = sc->v2 + a * tau - 0.5f * j * tau * tau;
        pos   = sc->p2 + sc->v2 * tau + 0.5f * a * tau * tau
                - (1.0f / 6.0f) * j * tau * tau * tau;
    } else if (t < sc->t4) {
        float tau = t - sc->t3;
        accel = 0.0f;
        vel   = sc->vlim;
        pos   = sc->p3 + sc->vlim * tau;
    } else if (t < sc->t5) {
        float tau = t - sc->t4;
        accel = -j * tau;
        vel   = sc->vlim - 0.5f * j * tau * tau;
        pos   = sc->p4 + sc->vlim * tau - (1.0f / 6.0f) * j * tau * tau * tau;
    } else if (t < sc->t6) {
        float tau = t - sc->t5;
        accel = -a;
        vel   = sc->v5 - a * tau;
        pos   = sc->p5 + sc->v5 * tau - 0.5f * a * tau * tau;
    } else {
        float tau = t - sc->t6;
        accel = -a + j * tau;
        vel   = sc->v6 - a * tau + 0.5f * j * tau * tau;
        pos   = sc->p6 + sc->v6 * tau - 0.5f * a * tau * tau
                + (1.0f / 6.0f) * j * tau * tau * tau;
    }

    sc->pos         = sc->start_pos + sc->direction * pos;
    sc->velocity    = sc->direction * vel;
    sc->acceleration = sc->direction * accel;

    return 0;
}

void s_curve_set_v_max(s_curve_t *sc, float v_max) {
  sc->v_max = v_max;
}

void s_curve_set_a_max(s_curve_t *sc, float a_max) {
  sc->a_max = a_max;
}

void s_curve_set_j_max(s_curve_t *sc, float j_max) {
  sc->j_max = j_max;
}

float s_curve_get_pos(s_curve_t *sc)   { return sc->pos; }
float s_curve_get_vel(s_curve_t *sc)   { return sc->velocity; }
float s_curve_get_accel(s_curve_t *sc) { return sc->acceleration; }
float s_curve_get_duration(const s_curve_t *sc) { return sc->t7; }
