#include "self_commissioning.h"
#include <string.h>

void sc_init(self_commissioning_t *sc, foc_t *hfoc) {
    sc->p_foc = hfoc;
    sc->signal_t = 0;
    sc->signal_flag = 0;
}

int sc_start_signal_dc(self_commissioning_t *sc, float dc) {
    if (sc->signal_flag) return -1;
    sc->signal_offset = dc;
    sc->signal_t = 0;
    sc->signal_start_t = 0;
    sc->signal_delay_t = 0.2f/(1.0f/20000.0f);
    memset(sc->v_buffer, 0, sizeof(sc->v_buffer));
    memset(sc->i_buffer, 0, sizeof(sc->i_buffer));
    sc->signal_flag = 1;
    return 0;
}

int sc_start_signal_sinusoidal(self_commissioning_t *sc, float amp, float freq, float offset) {
    if (sc->signal_flag) return -1;
    sc->signal_amplitude = amp;
    sc->signal_omega = TWO_PI * freq;
    sc->signal_offset = offset;
    sc->signal_t = 0;
    sc->signal_start_t = 0;
    sc->signal_delay_t = 0.2f/(1.0f/20000.0f);
    memset(sc->v_buffer, 0, sizeof(sc->v_buffer));
    memset(sc->i_buffer, 0, sizeof(sc->i_buffer));
    sc->signal_flag = 1;
    return 0;
}

int sc_start_measure_motor_resistance(self_commissioning_t *sc) {
    if (sc_start_signal_dc(sc, 1.0f) != 0) return -1;
    foc_set_motor_mode(sc->p_foc, MOTOR_MODE_VOLTAGE_CONTROL);
    sc->seq = SC_SEQUENCE_START_MEASURE_RS;
    return 0;
}

int sc_start_measure_motor_Ld(self_commissioning_t *sc) {
    if (sc_start_signal_sinusoidal(sc, 1.0f, 1000.0f, 1.0f) != 0) return -1;
    foc_set_motor_mode(sc->p_foc, MOTOR_MODE_VOLTAGE_CONTROL);
    sc->seq = SC_SEQUENCE_START_MEASURE_LD;
    return 0;
}

int sc_start_measure_motor_Lq(self_commissioning_t *sc) {
    if (sc_start_signal_sinusoidal(sc, 1.0f, 1000.0f, 0.0f) != 0) return -1;
    foc_set_motor_mode(sc->p_foc, MOTOR_MODE_VOLTAGE_CONTROL);
    sc->seq = SC_SEQUENCE_START_MEASURE_LQ;
    return 0;
}

int sc_start_calibrate_abs_encoder(self_commissioning_t *sc) {
    if (sc->signal_flag) return -1;
    sc->signal_calibrate_encoder_idx = 0;
    sc->signal_t = 0;
    sc->last_signal_t = 0;
    sc->signal_start_t = 0;
    sc->signal_delay_t = 0.2f/(1.0f/20000.0f);
    if (sc->p_foc->p_abs_encoder_error_comp_deg) {
        for (int i = 0; i < ERROR_LUT_SIZE; i++) {
            sc->p_foc->p_abs_encoder_error_comp_deg[i] = 0;
        }
    }
    foc_set_motor_mode(sc->p_foc, MOTOR_MODE_VOLTAGE_CONTROL);
    sc->seq = SC_SEQUENCE_START_CALIBRATE_ABS_ENCODER;
    sc->signal_flag = 1;
    return 0;
}

float sc_estimate_resistance(self_commissioning_t *sc) {
    float mean_vd = 0, mean_id = 0;

    for (int i = 0; i < MAX_DATA_ACQ_BUFFER; i++) {
        mean_vd += sc->v_buffer[i];
        mean_id += sc->i_buffer[i];
    }
    mean_vd /= MAX_DATA_ACQ_BUFFER;
    mean_id /= MAX_DATA_ACQ_BUFFER;

    float Rs = mean_vd / mean_id;
    return fabsf(Rs);
}

float sc_estimate_inductance(self_commissioning_t *sc, float Ts) {
    float Vc = 0, Vs = 0, Ic = 0, Is = 0;
    float mean_v = 0, mean_i = 0;

    // Remove DC offset
    for (int i = 0; i < MAX_DATA_ACQ_BUFFER; i++) {
        mean_v += sc->v_buffer[i];
        mean_i += sc->i_buffer[i];
    }
    mean_v /= MAX_DATA_ACQ_BUFFER;
    mean_i /= MAX_DATA_ACQ_BUFFER;

    // Single-frequency DFT
    for (int i = 0; i < MAX_DATA_ACQ_BUFFER; i++) {
        float angle = sc->signal_omega * i * Ts;

        float voltage = sc->v_buffer[i] - mean_v;
        float current = sc->i_buffer[i] - mean_i;

        Vc += voltage * fast_cos(angle);
        Vs += voltage * fast_sin(angle);
        Ic += current * fast_cos(angle);
        Is += current * fast_sin(angle);
    }

    float norm = 2.0f / MAX_DATA_ACQ_BUFFER;
    Vc *= norm; 
    Vs *= norm;
    Ic *= norm; 
    Is *= norm;

    // V & I amplitude
    float V_mag = sqrtf(Vc * Vc + Vs * Vs);
    float I_mag = sqrtf(Ic * Ic + Is * Is);

    // (phi = arctan(Vs/Vc) - arctan(Is/Ic))
    float phi = atan2f(Vs, Vc) - atan2f(Is, Ic);

    // Impedansi & parameters
    float Z_mag = V_mag / I_mag;

    float L_est = (Z_mag * fast_sin(phi)) / sc->signal_omega;
    return fabsf(L_est);
}

void sc_measure_Rs_update(self_commissioning_t *sc) {
    float vd = 0, vq = 0;
    float id = 0, iq = 0;

    uint32_t idx = 0;
    if (sc->signal_t < sc->signal_delay_t) {
        sc->signal_start_t = sc->signal_t;
    }
    else {
        idx = sc->signal_t - sc->signal_start_t;
    }
    foc_get_idiq(sc->p_foc, &id, &iq);
    
    if (idx >= MAX_DATA_ACQ_BUFFER) {
        sc->i_buffer[idx - 1] = id;
        sc->Rs = sc_estimate_resistance(sc);
        foc_set_open_loop_voltage(sc->p_foc, 0, 0, 0);
        sc->signal_flag = 0;
        sc->measure_done_flag = 1;
        return;
    }

    vd = sc->signal_offset;
    sc->v_buffer[idx] = vd;
    if (idx > 0) {
        sc->i_buffer[idx - 1] = id;
    }
    foc_set_open_loop_voltage(sc->p_foc, vd, vq, 0);
    sc->signal_t++;
}

void sc_measure_Ld_update(self_commissioning_t *sc, float Ts) {
    float vd = 0, vq = 0;
    float id = 0, iq = 0;

    uint32_t idx = 0;
    if (sc->signal_t < sc->signal_delay_t) {
        sc->signal_start_t = sc->signal_t;
    }
    else {
        idx = sc->signal_t - sc->signal_start_t;
    }
    foc_get_idiq(sc->p_foc, &id, &iq);
    
    if (idx >= MAX_DATA_ACQ_BUFFER) {
        sc->i_buffer[idx - 1] = id;
        sc->Ld = sc_estimate_inductance(sc, Ts);
        foc_set_open_loop_voltage(sc->p_foc, 0, 0, 0);
        sc->signal_flag = 0;
        sc->measure_done_flag = 1;
        return;
    }

    float rad = sc->signal_omega * (float)sc->signal_t * Ts;
    vd = sc->signal_amplitude * fast_sin(rad);
    sc->v_buffer[idx] = vd;
    if (idx > 0) {
        sc->i_buffer[idx - 1] = id;
    }
    foc_set_open_loop_voltage(sc->p_foc, vd, vq, 0);
    sc->signal_t++;
}

void sc_measure_Lq_update(self_commissioning_t *sc, float Ts) {
    float vd = 0, vq = 0;
    float id = 0, iq = 0;

    uint32_t idx = 0;
    if (sc->signal_t < sc->signal_delay_t) {
        sc->signal_start_t = sc->signal_t;
    }
    else {
        idx = sc->signal_t - sc->signal_start_t;
    }
    foc_get_idiq(sc->p_foc, &id, &iq);
    
    if (idx >= MAX_DATA_ACQ_BUFFER) {
        sc->i_buffer[idx - 1] = iq;
        sc->Lq = sc_estimate_inductance(sc, Ts);
        foc_set_open_loop_voltage(sc->p_foc, 0, 0, 0);
        sc->signal_flag = 0;
        sc->measure_done_flag = 1;
        return;
    }

    float rad = sc->signal_omega * (float)sc->signal_t * Ts;
    vq = sc->signal_amplitude * fast_sin(rad);
    vd = 1.0f;
    sc->v_buffer[idx] = vq;
    if (idx > 0) {
        sc->i_buffer[idx - 1] = iq;
    }
    foc_set_open_loop_voltage(sc->p_foc, vd, vq, 0);
    sc->signal_t++;
}

void sc_calibrate_abs_encoder_update(self_commissioning_t *sc) {
    float vd = 1.5f;
    float vq = 0.0f;
    
    float mech_degree = sc->p_foc->get_mech_degre();

    if (sc->signal_t < sc->signal_delay_t) {
        sc->abs_encoder_deg_initial = mech_degree;
        sc->signal_calibrate_encoder_idx = 0;
    }
    else {
        if (sc->signal_t - sc->last_signal_t >= 50) {
            sc->last_signal_t = sc->signal_t;
            float actual_degree = (float)sc->signal_calibrate_encoder_idx / ERROR_LUT_SIZE * 360.0f;
            if (sc->p_foc->p_abs_encoder_error_comp_deg) {
                float lut_idx_f = (mech_degree / 360.0f) * ERROR_LUT_SIZE;
                lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE);
                if (lut_idx_f < 0) {
                    lut_idx_f += ERROR_LUT_SIZE;
                }
                
                int idx = (int)lut_idx_f;
                if (idx >= 0 && idx < ERROR_LUT_SIZE) {
                    sc->p_foc->p_abs_encoder_error_comp_deg[idx] = actual_degree;
                }
            }
            sc->signal_calibrate_encoder_idx++;
            if (sc->signal_calibrate_encoder_idx >= ERROR_LUT_SIZE) {
                foc_set_open_loop_voltage(sc->p_foc, 0, 0, 0);
                sc->signal_flag = 0;
                sc->measure_done_flag = 1;
                
                // check empty value
                float *abs_error_comp = sc->p_foc->p_abs_encoder_error_comp_deg;
                if (abs_error_comp) {
                    for (int i = 0; i < ERROR_LUT_SIZE; i++) {
                        if (abs_error_comp[i] == 0) {
                            int last_i = i - 1;
                            int next_i = i + 1;
                            if (last_i < 0) last_i += ERROR_LUT_SIZE;
                            if (next_i > ERROR_LUT_SIZE) next_i -= ERROR_LUT_SIZE;
                            abs_error_comp[i] = (abs_error_comp[last_i] + abs_error_comp[next_i]) / 2.0f;
                        }
                    }
                }
                return;
            }
        }
    }

    float e_rad = (float)sc->signal_calibrate_encoder_idx / ERROR_LUT_SIZE * TWO_PI * sc->p_foc->pole_pairs;
    foc_set_open_loop_voltage(sc->p_foc, vd, vq, e_rad);
    sc->signal_t++;
}

void sc_update(self_commissioning_t *sc, float Ts) {
    if (!sc->signal_flag) return;

    switch (sc->seq) {
        case SC_SEQUENCE_START_MEASURE_RS: {
            sc_measure_Rs_update(sc);
            break;
        }
        case SC_SEQUENCE_START_MEASURE_LD: {
            sc_measure_Ld_update(sc, Ts);
            break;
        }
        case SC_SEQUENCE_START_MEASURE_LQ: {
            sc_measure_Lq_update(sc, Ts);
            break;
        }
        case SC_SEQUENCE_START_CALIBRATE_ABS_ENCODER: {
            sc_calibrate_abs_encoder_update(sc);
            break;
        }
        default:
            break;
    }
}

_Bool sc_is_measure_done(self_commissioning_t *sc) {
    if (sc->measure_done_flag) {
        sc->measure_done_flag = 0;
        return 1;
    }
    return 0;
}

sc_sequence_t sc_get_seq(self_commissioning_t *sc) {
    return sc->seq;
}

float sc_get_Rs(self_commissioning_t *sc) {
    return sc->Rs;
}

float sc_get_Ld(self_commissioning_t *sc) {
    return sc->Ld;
}

float sc_get_Lq(self_commissioning_t *sc) {
    return sc->Lq;
}

