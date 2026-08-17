/*
 * CAN.h
 *
 *  Created on: October 14, 2025
 *      Author: munir
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

typedef enum {
    CAN_SEQ_IDLE,
    CAN_SEQ_FIRST_FRAME,
    CAN_SEQ_MIDLE_FRAME,
    CAN_SEQ_LAST_FRAME
} can_sequence_t;

typedef struct {
    uint32_t id;
    uint32_t start_of_frame;
    uint16_t total_data_length;
    uint16_t remaining_data;
    uint16_t segment;
    uint8_t *data;
    uint32_t end_of_frame;
} can_frame_t;

typedef struct {
    int (*init)(void);
    int (*send_data)(uint32_t, uint8_t*, uint32_t);
    int (*recv_data)(uint32_t*, uint8_t*, uint32_t*);
    _Bool (*is_mailboxes_free)(void);
    uint32_t (*get_tick_ms)(void);
} can_config_t;

typedef struct {
    uint8_t frame_recv[8];
    uint32_t frame_recv_length;
    _Bool incomming_frame_flag;

    can_sequence_t tx_seq;
    can_sequence_t rx_seq;
    can_frame_t tx_frame;
    can_frame_t rx_frame;
    _Bool send_frame_flag;
    uint32_t tick_ms;

    can_config_t config;
} can_protocol_t;

void can_motor_init(can_protocol_t *can, can_config_t config, uint8_t *data_tx_buffer, uint8_t *data_rx_buffer);
int can_motor_start_send_data(can_protocol_t *can, uint32_t id, uint8_t *data, uint16_t data_length);
int can_motor_recv_chunked_frame(can_protocol_t *can);
void can_motor_send_frame_update(can_protocol_t *can);
int can_motor_recv_frame_update(can_protocol_t *can);

#endif
