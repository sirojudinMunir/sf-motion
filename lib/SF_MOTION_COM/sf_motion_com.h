#ifndef SF_MOTION_COM_H
#define SF_MOTION_COM_H


#include <stdint.h>
#include "string.h"
#include "motor.h"
#include "AS5047P.h"
#include "FOC_utils.h"
#include "self_commissioning.h"
#include "storage.h"

#define SFM_COM_COMTYPE_INDEX    0
#define SFM_COM_DEVICE_ID_INDEX  1
#define SFM_COM_ADDRESS_INDEX    2
#define SFM_COM_HEADER_LENGTH    3 // (com_type(1-byte) + device_id(1-byte) + address(1-byte))
#define SFM_COM_DATA_OFFSET      SFM_COM_HEADER_LENGTH


#define MAX_PLOTTER_LINE 10

typedef enum {
  SFM_COM_TYPE_RESPONSE,
  SFM_COM_TYPE_READ,
  SFM_COM_TYPE_WRITE,
  SFM_COM_TYPE_ENABLE_STREAMING,
  SFM_COM_TYPE_DISABLE_STREAMING,
  SFM_COM_TYPE_STREAMING,
  SFM_COM_TYPE_EVENT,
}sfm_com_type_t;

typedef enum {
  SFM_COM_STREAM_DIS = 0,
  SFM_COM_STREAM_ENA = 1,
} sfm_com_streaming_t;

typedef struct {
  uint8_t addr;
  uint8_t data[8];
  uint16_t data_len;
  _Bool send_flag;
}sfm_plotter_t;

typedef struct {
  _Bool incomming_data_flag;
  uint8_t data_rx[32];
  uint32_t data_rx_len;

  uint8_t data_tx[32];
  uint16_t data_tx_len;
  _Bool send_data_flag;

  sfm_plotter_t plotter[MAX_PLOTTER_LINE];
  uint8_t plotter_line;

  uint32_t plotter_tick;
  foc_t *pfoc;
  storage_t *pstorage;
  self_commissioning_t *psc;

  int (*recv_data)(uint8_t*, uint16_t);
  int (*send_data)(uint8_t*, uint16_t);
  uint32_t (*get_tick_ms)(void);
}sfm_com_t;

typedef int8_t (*sfm_com_read_t)(sfm_com_t *com, void *data);
typedef int8_t (*sfm_com_write_t)(sfm_com_t *com, void *data);

void sfm_com_init(sfm_com_t *com, int (*recv_data)(uint8_t*, uint16_t), int (*send_data)(uint8_t*, uint16_t), uint32_t (*get_tick_ms)(void),
                  foc_t *pfoc, storage_t *pstorage, self_commissioning_t *psc);
void sfm_com_update(sfm_com_t *com);


#endif