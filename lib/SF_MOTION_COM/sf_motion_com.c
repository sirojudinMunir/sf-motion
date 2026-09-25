#include "sf_motion_com.h"

#define SFM_GETTER(name, get)                     \
static int8_t name(sfm_com_t *com, void *data) {  \
  get;                                            \
  return 0;                                       \
}

#define SFM_SETTER(name, set)                    \
static int8_t name(sfm_com_t *com, void *data) { \
  set;                                           \
  return 0;                                      \
}


int8_t sfm_com_send_data(sfm_com_t *com, sfm_com_type_t com_type, uint8_t address, void *data, uint16_t size_of_data) {
  if (com->send_data_flag) return -1;
  com->data_tx_len = size_of_data + SFM_COM_HEADER_LENGTH;
  com->data_tx[SFM_COM_COMTYPE_INDEX] = (uint8_t)com_type;
  com->data_tx[SFM_COM_DEVICE_ID_INDEX] = (uint8_t)(com->pfoc->motor.device_id & 0xff);
  com->data_tx[SFM_COM_ADDRESS_INDEX] = address;
  memcpy(com->data_tx + SFM_COM_DATA_OFFSET, data, size_of_data);
  com->send_data_flag = 1;
  return 0;
}

int8_t sfm_com_send_data_plotter(sfm_com_t *com, sfm_plotter_t *plotter, void *data, uint16_t size_of_data) {
  if (plotter->send_flag) return -1;
  plotter->data_len = size_of_data + SFM_COM_HEADER_LENGTH;
  plotter->data[SFM_COM_COMTYPE_INDEX] = (uint8_t)SFM_COM_TYPE_STREAMING;
  plotter->data[SFM_COM_DEVICE_ID_INDEX] = (uint8_t)(com->pfoc->motor.device_id & 0xff);
  plotter->data[SFM_COM_ADDRESS_INDEX] = plotter->addr;
  memcpy(plotter->data + SFM_COM_DATA_OFFSET, data, size_of_data);
  plotter->send_flag = 1;
  return 0;
}

sfm_com_type_t sfm_com_get_com_type(sfm_com_t *com) {
  return com->data_rx[SFM_COM_COMTYPE_INDEX];
}

uint8_t sfm_com_get_id(sfm_com_t *com) {
  return com->data_rx[SFM_COM_DEVICE_ID_INDEX];
}

uint8_t sfm_com_get_address(sfm_com_t *com) {
  return com->data_rx[SFM_COM_ADDRESS_INDEX];
}

int8_t sfm_com_recv_data(sfm_com_t *com, void *data, uint16_t size_of_data) {
  int8_t ret_val = -1;
  if (com->data_rx_len == size_of_data + SFM_COM_HEADER_LENGTH) {
    memcpy(data, &com->data_rx[SFM_COM_DATA_OFFSET], size_of_data);
    ret_val = 0;
  }
  return ret_val;
}

int8_t sfm_com_handle_parameter(
  sfm_com_t *com,
  sfm_com_read_t com_read,
  sfm_com_write_t com_write,
  sfm_com_streaming_t streaming,
  void *data,
  uint16_t size)
{
  sfm_com_type_t com_type = sfm_com_get_com_type(com);
  uint8_t addr = sfm_com_get_address(com);
  int8_t ret_val = -1;

  switch (com_type) {
    case SFM_COM_TYPE_READ: {
      if (com_read) {
        if (com_read(com, data) == 0) {
          sfm_com_send_data(com, SFM_COM_TYPE_RESPONSE, addr, data, size);
          return 0;
        }
      }
      break;
    }

    case SFM_COM_TYPE_WRITE: {
      if (com_write && sfm_com_recv_data(com, data, size) == 0) {
        ret_val = com_write(com, data);
      }
      break;
    }

    case SFM_COM_TYPE_ENABLE_STREAMING: {
      if (streaming == SFM_COM_STREAM_ENA) {
        _Bool already_enabled = 0;
        for (int i = 0; i < com->plotter_line; i++) {
          if (com->plotter[i].addr == addr) {
            already_enabled = 1;
            break;
          }
        }
        if (!already_enabled && com->plotter_line < MAX_PLOTTER_LINE) {
          com->plotter[com->plotter_line].addr = addr;
          com->plotter_line++;
          ret_val = 0;
        }
      }
      break;
    }

    case SFM_COM_TYPE_DISABLE_STREAMING: {
      if (streaming == SFM_COM_STREAM_ENA) {
        int start_idx = -1;
        for (int i = 0; i < com->plotter_line; i++) {
          if (com->plotter[i].addr == addr) {
            start_idx = i;
            break;
          }
        }
        if (start_idx >= 0) {
          for (int i = start_idx; i < com->plotter_line - 1; i++) {
            com->plotter[i].addr = com->plotter[i + 1].addr;
          }
          com->plotter_line--;
          com->plotter[com->plotter_line].addr = 0;
          ret_val = 0;
        }
      }
      break;
    }

    default: {
      ret_val = -2;
      break;
    }
  }

  sfm_com_send_data(com, SFM_COM_TYPE_RESPONSE, addr, &ret_val, sizeof(ret_val));
  return ret_val;
}

/************************************************************************************************ */

static int8_t get_version(sfm_com_t *com, void *data) {
  uint8_t version[3] = {
    SF_MOTION_MAJOR_VERSION,
    SF_MOTION_MINOR_VERSION,
    SF_MOTION_PATCH_VERSION,
  };
  memcpy(data, version, sizeof(version));
  return 0;
}

SFM_GETTER(get_device_id, *(uint8_t *)data = (uint8_t)com->pstorage->memory.general.device_id)
SFM_SETTER(set_device_id, com->pstorage->memory.general.device_id = (uint32_t)*(uint8_t *)data)

static int8_t set_default_config(sfm_com_t *com, void *data) {
  (void) data;
  storage_default_config(com->pstorage);
  storage_copy_to_local(com->pstorage, com->pfoc);
  return 0;
}

static int8_t set_save_config(sfm_com_t *com, void *data) {
  (void) data;
  storage_copy_from_local(com->pstorage, com->pfoc);
  if (storage_save_config(com->pstorage) != 0) {
    return -1;
  }
  return 0;
}

static int8_t set_start_measure_resistance(sfm_com_t *com, void *data) {
  (void) data;
  if (sc_start_measure_motor_resistance(com->psc) != 0) return -1;
  return 0;
}

static int8_t set_start_measure_ld(sfm_com_t *com, void *data) {
  (void) data;
  if (sc_start_measure_motor_Ld(com->psc) != 0) return -1;
  return 0;
}

static int8_t set_start_measure_lq(sfm_com_t *com, void *data) {
  (void) data;
  if (sc_start_measure_motor_Lq(com->psc) != 0) return -1;
  return 0;
}

static int8_t set_start_calibrate_abs_encoder(sfm_com_t *com, void *data) {
  (void) data;
  if (sc_start_calibrate_abs_encoder(com->psc) != 0) return -1;
  return 0;
}

/************************************************************************************************ */

SFM_GETTER(get_foc_mode, *(foc_mode_t *)data = foc_get_mode(com->pfoc))
SFM_SETTER(set_foc_mode, foc_set_mode(com->pfoc, *(foc_mode_t *)data))

SFM_GETTER(get_motor_mode, *(motor_mode_t *)data = foc_get_motor_mode(com->pfoc))
SFM_SETTER(set_motor_mode, foc_set_motor_mode(com->pfoc, *(motor_mode_t *)data))

SFM_GETTER(get_pole_pairs, *(uint8_t *)data = foc_get_motor_pole_pairs(com->pfoc))
SFM_SETTER(set_pole_pairs, foc_set_motor_pole_pairs(com->pfoc, *(uint8_t *)data))

SFM_GETTER(get_kv, *(float *)data = foc_get_motor_kv(com->pfoc))
SFM_SETTER(set_kv, foc_set_motor_kv(com->pfoc, *(float *)data))

SFM_GETTER(get_Rs, *(float *)data = foc_get_motor_Rs(com->pfoc))
SFM_SETTER(set_Rs, foc_set_motor_Rs(com->pfoc, *(float *)data))

SFM_GETTER(get_Ld, *(float *)data = foc_get_motor_Ld(com->pfoc))
SFM_SETTER(set_Ld, foc_set_motor_Ld(com->pfoc, *(float *)data))

SFM_GETTER(get_Lq, *(float *)data = foc_get_motor_Lq(com->pfoc))
SFM_SETTER(set_Lq, foc_set_motor_Lq(com->pfoc, *(float *)data))

SFM_GETTER(get_flux_linkage, *(float *)data = foc_get_motor_flux_linkage(com->pfoc))
SFM_SETTER(set_flux_linkage, foc_set_motor_flux_linkage(com->pfoc, *(float *)data))

/************************************************************************************************ */

SFM_GETTER(get_id_kp, *(float *)data = pid_get_kp(&com->pfoc->id_ctrl))
SFM_SETTER(set_id_kp, pid_set_kp(&com->pfoc->id_ctrl, *(float *)data))

SFM_GETTER(get_id_ki, *(float *)data = pid_get_ki(&com->pfoc->id_ctrl))
SFM_SETTER(set_id_ki, pid_set_ki(&com->pfoc->id_ctrl, *(float *)data))

SFM_GETTER(get_id_deadband, *(float *)data = pid_get_deadband(&com->pfoc->id_ctrl))
SFM_SETTER(set_id_deadband, pid_set_deadband(&com->pfoc->id_ctrl, *(float *)data))

SFM_GETTER(get_iq_kp, *(float *)data = pid_get_kp(&com->pfoc->iq_ctrl))
SFM_SETTER(set_iq_kp, pid_set_kp(&com->pfoc->iq_ctrl, *(float *)data))

SFM_GETTER(get_iq_ki, *(float *)data = pid_get_ki(&com->pfoc->iq_ctrl))
SFM_SETTER(set_iq_ki, pid_set_ki(&com->pfoc->iq_ctrl, *(float *)data))

SFM_GETTER(get_iq_deadband, *(float *)data = pid_get_deadband(&com->pfoc->iq_ctrl))
SFM_SETTER(set_iq_deadband, pid_set_deadband(&com->pfoc->iq_ctrl, *(float *)data))

SFM_GETTER(get_speed_kp, *(float *)data = pid_get_kp(&com->pfoc->speed_ctrl))
SFM_SETTER(set_speed_kp, pid_set_kp(&com->pfoc->speed_ctrl, *(float *)data))

SFM_GETTER(get_speed_ki, *(float *)data = pid_get_ki(&com->pfoc->speed_ctrl))
SFM_SETTER(set_speed_ki, pid_set_ki(&com->pfoc->speed_ctrl, *(float *)data))

SFM_GETTER(get_speed_out_max, *(float *)data = pid_get_out_max(&com->pfoc->speed_ctrl))
SFM_SETTER(set_speed_out_max, pid_set_out_constraint(&com->pfoc->speed_ctrl, *(float *)data, -*(float *)data))

SFM_GETTER(get_speed_deadband, *(float *)data = pid_get_deadband(&com->pfoc->speed_ctrl))
SFM_SETTER(set_speed_deadband, pid_set_deadband(&com->pfoc->speed_ctrl, *(float *)data))

SFM_GETTER(get_position_kp, *(float *)data = pid_get_kp(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_kp, pid_set_kp(&com->pfoc->pos_ctrl, *(float *)data))

SFM_GETTER(get_position_ki, *(float *)data = pid_get_ki(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_ki, pid_set_ki(&com->pfoc->pos_ctrl, *(float *)data))

SFM_GETTER(get_position_kd, *(float *)data = pid_get_kd(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_kd, pid_set_kd(&com->pfoc->pos_ctrl, *(float *)data))

SFM_GETTER(get_position_out_max, *(float *)data = pid_get_out_max(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_out_max, pid_set_out_constraint(&com->pfoc->pos_ctrl, *(float *)data, -*(float *)data))

SFM_GETTER(get_position_deadband, *(float *)data = pid_get_deadband(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_deadband, pid_set_deadband(&com->pfoc->pos_ctrl, *(float *)data))

SFM_GETTER(get_position_d_filter_fc, *(float *)data = pid_get_d_filter_fc(&com->pfoc->pos_ctrl))
SFM_SETTER(set_position_d_filter_fc, pid_set_d_filter_fc(&com->pfoc->pos_ctrl, *(float *)data))

SFM_GETTER(get_fw_kp, *(float *)data = pid_get_kp(&com->pfoc->fw_ctrl))
SFM_SETTER(set_fw_kp, pid_set_kp(&com->pfoc->fw_ctrl, *(float *)data))

SFM_GETTER(get_fw_ki, *(float *)data = pid_get_ki(&com->pfoc->fw_ctrl))
SFM_SETTER(set_fw_ki, pid_set_ki(&com->pfoc->fw_ctrl, *(float *)data))

SFM_GETTER(get_fw_out_min, *(float *)data = pid_get_out_min(&com->pfoc->fw_ctrl))
SFM_SETTER(set_fw_out_min, pid_set_out_constraint(&com->pfoc->fw_ctrl, 0.0f, *(float *)data))

SFM_GETTER(get_fw_enable, *(uint8_t *)data = (uint8_t)foc_get_fw_enable(com->pfoc))
SFM_SETTER(set_fw_enable, foc_set_fw_enable(com->pfoc, (_Bool)*(uint8_t *)data))

SFM_GETTER(get_mtpa_enable, *(uint8_t *)data = (uint8_t)foc_get_mtpa_enable(com->pfoc))
SFM_SETTER(set_mtpa_enable, foc_set_mtpa_enable(com->pfoc, (_Bool)*(uint8_t *)data))

/************************************************************************************************ */

SFM_GETTER(get_current_set_point, *(float *)data = com->pfoc->Is_ref)
SFM_SETTER(set_current_set_point, foc_set_current_set_point(com->pfoc, *(float *)data))

SFM_GETTER(get_speed_set_point, *(float *)data = com->pfoc->rpm_ref)
SFM_SETTER(set_speed_set_point, foc_set_speed_set_point(com->pfoc, *(float *)data))

SFM_GETTER(get_position_set_point, *(float *)data = com->pfoc->pos_ref)
SFM_SETTER(set_position_set_point, foc_set_position_set_point(com->pfoc, *(float *)data))

SFM_GETTER(get_ia, *(float *)data = com->pfoc->ia)
SFM_GETTER(get_ib, *(float *)data = com->pfoc->ib)
SFM_GETTER(get_ic, *(float *)data = com->pfoc->ic)
SFM_GETTER(get_i_alpha, *(float *)data = com->pfoc->i_alpha)
SFM_GETTER(get_i_beta, *(float *)data = com->pfoc->i_beta)
SFM_GETTER(get_id, *(float *)data = com->pfoc->id)
SFM_GETTER(get_iq, *(float *)data = com->pfoc->iq)

SFM_GETTER(get_va, *(float *)data = com->pfoc->va)
SFM_GETTER(get_vb, *(float *)data = com->pfoc->vb)
SFM_GETTER(get_vc, *(float *)data = com->pfoc->vc)
SFM_GETTER(get_v_alpha, *(float *)data = com->pfoc->v_alpha)
SFM_GETTER(get_v_beta, *(float *)data = com->pfoc->v_beta)

SFM_GETTER(get_vd, *(float *)data = com->pfoc->vd)
SFM_SETTER(set_vd, com->pfoc->vd = *(float *)data)

SFM_GETTER(get_vq, *(float *)data = com->pfoc->vq)
SFM_SETTER(set_vq, com->pfoc->vq = *(float *)data)

SFM_GETTER(get_e_rad, *(float *)data = com->pfoc->e_rad)
SFM_SETTER(set_e_rad, com->pfoc->e_rad = *(float *)data)

SFM_GETTER(get_actual_rpm, *(float *)data = com->pfoc->actual_rpm)
SFM_GETTER(get_actual_angle, *(float *)data = com->pfoc->actual_angle)

SFM_GETTER(get_Is_ref, *(float *)data = com->pfoc->Is_ref)
SFM_GETTER(get_rpm_ref, *(float *)data = com->pfoc->rpm_ref)
SFM_GETTER(get_pos_ref, *(float *)data = com->pfoc->pos_ref)

SFM_GETTER(get_m_angle_rad, *(float *)data = com->pfoc->m_angle_rad)
SFM_GETTER(get_m_angle_rad_comp, *(float *)data = com->pfoc->m_angle_rad_comp)
SFM_GETTER(get_v_bus, *(float *)data = com->pfoc->v_bus)

/************************************************************************************************ */

typedef struct {
  sfm_com_read_t com_read;
  sfm_com_write_t com_write;
  sfm_com_streaming_t streaming;
  uint16_t value_length;
}sfm_com_register_handler_t;

sfm_com_register_handler_t sfm_com_reg[] = {
  {NULL, NULL, SFM_COM_STREAM_DIS, 0},                                                      // 0
  {get_version, NULL, SFM_COM_STREAM_DIS, 3},                                               // 1
  {get_device_id, set_device_id, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                      // 2
  {NULL, NULL, SFM_COM_STREAM_DIS, 0},                                                      // 3
  {NULL, NULL, SFM_COM_STREAM_DIS, 0},                                                      // 4
  {NULL, NULL, SFM_COM_STREAM_DIS, 0},                                                      // 5
  {NULL, NULL, SFM_COM_STREAM_DIS, 0},                                                      // 6
  {NULL, set_default_config, SFM_COM_STREAM_DIS, 0},                                        // 7
  {NULL, set_save_config, SFM_COM_STREAM_DIS, 0},                                           // 8
  {NULL, set_start_measure_resistance, SFM_COM_STREAM_DIS, 0},                              // 9
  {NULL, set_start_measure_ld, SFM_COM_STREAM_DIS, 0},                                      // 10
  {NULL, set_start_measure_lq, SFM_COM_STREAM_DIS, 0},                                      // 11
  {NULL, set_start_calibrate_abs_encoder, SFM_COM_STREAM_DIS, 0},                           // 12
  
  {get_foc_mode, set_foc_mode, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                        // 13
  {get_motor_mode, set_motor_mode, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                    // 14
  {get_pole_pairs, set_pole_pairs, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                    // 15
  {get_kv, set_kv, SFM_COM_STREAM_DIS, sizeof(float)},                                      // 16
  {get_Rs, set_Rs, SFM_COM_STREAM_DIS, sizeof(float)},                                      // 17
  {get_Ld, set_Ld, SFM_COM_STREAM_DIS, sizeof(float)},                                      // 18
  {get_Lq, set_Lq, SFM_COM_STREAM_DIS, sizeof(float)},                                      // 19
  {get_flux_linkage, set_flux_linkage, SFM_COM_STREAM_DIS, sizeof(float)},                  // 20
       
  {get_id_kp, set_id_kp, SFM_COM_STREAM_DIS, sizeof(float)},                                // 21
  {get_id_ki, set_id_ki, SFM_COM_STREAM_DIS, sizeof(float)},                                // 22
  {get_id_deadband, set_id_deadband, SFM_COM_STREAM_DIS, sizeof(float)},                    // 23
  {get_iq_kp, set_iq_kp, SFM_COM_STREAM_DIS, sizeof(float)},                                // 24
  {get_iq_ki, set_iq_ki, SFM_COM_STREAM_DIS, sizeof(float)},                                // 25
  {get_iq_deadband, set_iq_deadband, SFM_COM_STREAM_DIS, sizeof(float)},                    // 26
  {get_speed_kp, set_speed_kp, SFM_COM_STREAM_DIS, sizeof(float)},                          // 27
  {get_speed_ki, set_speed_ki, SFM_COM_STREAM_DIS, sizeof(float)},                          // 28
  {get_speed_out_max, set_speed_out_max, SFM_COM_STREAM_DIS, sizeof(float)},                // 29
  {get_speed_deadband, set_speed_deadband, SFM_COM_STREAM_DIS, sizeof(float)},              // 30
  {get_position_kp, set_position_kp, SFM_COM_STREAM_DIS, sizeof(float)},                    // 31
  {get_position_ki, set_position_ki, SFM_COM_STREAM_DIS, sizeof(float)},                    // 32
  {get_position_kd, set_position_kd, SFM_COM_STREAM_DIS, sizeof(float)},                    // 33
  {get_position_out_max, set_position_out_max, SFM_COM_STREAM_DIS, sizeof(float)},          // 34
  {get_position_deadband, set_position_deadband, SFM_COM_STREAM_DIS, sizeof(float)},        // 35
  {get_position_d_filter_fc, set_position_d_filter_fc, SFM_COM_STREAM_DIS, sizeof(float)},  // 36
  {get_fw_kp, set_fw_kp, SFM_COM_STREAM_DIS, sizeof(float)},                                // 37
  {get_fw_ki, set_fw_ki, SFM_COM_STREAM_DIS, sizeof(float)},                                // 38
  {get_fw_out_min, set_fw_out_min, SFM_COM_STREAM_DIS, sizeof(float)},                      // 39
  {get_fw_enable, set_fw_enable, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                      // 40
  {get_mtpa_enable, set_mtpa_enable, SFM_COM_STREAM_DIS, sizeof(uint8_t)},                  // 41

  {get_current_set_point, set_current_set_point, SFM_COM_STREAM_ENA, sizeof(float)},        // 42
  {get_speed_set_point, set_speed_set_point, SFM_COM_STREAM_ENA, sizeof(float)},            // 43
  {get_position_set_point, set_position_set_point, SFM_COM_STREAM_ENA, sizeof(float)},      // 44
  {get_ia, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 45
  {get_ib, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 46
  {get_ic, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 47
  {get_i_alpha, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                   // 48
  {get_i_beta, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                    // 49
  {get_id, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 50
  {get_iq, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 51
  {get_va, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 52
  {get_vb, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 53
  {get_vc, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                        // 54
  {get_v_alpha, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                   // 55
  {get_v_beta, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                    // 56
  {get_vd, set_vd, SFM_COM_STREAM_ENA, sizeof(float)},                                      // 57
  {get_vq, set_vq, SFM_COM_STREAM_ENA, sizeof(float)},                                      // 58
  {get_e_rad, set_e_rad, SFM_COM_STREAM_ENA, sizeof(float)},                                // 59
  {get_actual_rpm, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                // 60
  {get_actual_angle, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                              // 61
  {get_Is_ref, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                    // 62
  {get_rpm_ref, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                   // 63
  {get_pos_ref, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                   // 64
  {get_m_angle_rad, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                               // 65
  {get_m_angle_rad_comp, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                          // 66
  {get_v_bus, NULL, SFM_COM_STREAM_ENA, sizeof(float)},                                     // 67
};

const int SFM_COM_MAX = sizeof(sfm_com_reg) / sizeof(sfm_com_register_handler_t);

static int8_t sfm_com_handle_all_parameters(sfm_com_t *com) {
  uint8_t value[4];
  int idx = sfm_com_get_address(com);
  if (idx >= SFM_COM_MAX) {
    return -1;
  }
  
  return sfm_com_handle_parameter(com, 
    sfm_com_reg[idx].com_read, 
    sfm_com_reg[idx].com_write, 
    sfm_com_reg[idx].streaming, 
    value, sfm_com_reg[idx].value_length
  );
}

/****************************************************************************** */

void sfm_com_init(sfm_com_t *com, int (*recv_data)(uint8_t*, uint16_t), int (*send_data)(uint8_t*, uint16_t), uint32_t (*get_tick_ms)(void),
                  foc_t *pfoc, storage_t *pstorage, self_commissioning_t *psc) {
  com->recv_data = recv_data;
  com->send_data = send_data;
  com->get_tick_ms = get_tick_ms;
  com->pfoc = pfoc;
  com->pstorage = pstorage;
  com->psc = psc;

  com->plotter_tick = com->get_tick_ms();
}

void sfm_com_update(sfm_com_t *com) {
  if (com->incomming_data_flag) {
    com->incomming_data_flag = 0;
    sfm_com_handle_all_parameters(com);
  }
  else {
    if (com->get_tick_ms() - com->plotter_tick >= 10) {
      com->plotter_tick = com->get_tick_ms();
      for (int i = 0; i < com->plotter_line; i++) {
        uint8_t plotter_addr = com->plotter[i].addr;
        if (plotter_addr > 0) {
          float value;
          sfm_com_reg[plotter_addr].com_read(com, &value);
          sfm_com_send_data_plotter(com, &com->plotter[i], &value, sizeof(value));
        }
      }
    }
  }
  if (com->send_data_flag) {
    if (com->send_data(com->data_tx, com->data_tx_len) == 0) {
      com->send_data_flag = 0;
    }
  }

  // plotter execute
  for (int i = 0; i < com->plotter_line; i++) {
    if (com->plotter[i].send_flag) {
      if (com->send_data(com->plotter[i].data, com->plotter[i].data_len) == 0) {
        com->plotter[i].send_flag = 0;
      }
    }
  }
}
