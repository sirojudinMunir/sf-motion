#include "CAN.h"
#include "string.h"


/******************************************************************************/

void can_motor_init(can_protocol_t *can, can_config_t config, uint8_t *data_tx_buffer, uint8_t *data_rx_buffer) {
    can->config = config;
    can->config.init();
    can->rx_seq = CAN_SEQ_FIRST_FRAME;
    can->tx_frame.data = data_tx_buffer;
    can->rx_frame.data = data_rx_buffer;
}

int can_motor_start_send_data(can_protocol_t *can, uint32_t id, uint8_t *data, uint16_t data_length) {
    if (can->tx_seq != CAN_SEQ_IDLE) return -1;
    can->tx_frame.start_of_frame = 0xDEADBEEF;
    can->tx_frame.id = id;
    can->tx_frame.data = data;
    can->tx_frame.total_data_length = data_length;
    can->tx_frame.end_of_frame = 0x5F0C5F0C;
    can->tx_frame.remaining_data = data_length;
    can->tx_frame.segment = 0;
    can->tx_seq = CAN_SEQ_FIRST_FRAME;
    return 0;
}

int can_motor_recv_chunked_frame(can_protocol_t *can) {
    uint32_t id = 0;
    if (can->config.recv_data(&id, can->frame_recv, &can->frame_recv_length) == 0) {
        can->incomming_frame_flag = 1;
        return 0;
    }
    return -1;
}

void can_motor_send_frame_update(can_protocol_t *can) {
#if 1
    if (can->config.get_tick_ms() - can->tick_ms < 1) return;
    can->tick_ms = can->config.get_tick_ms();
    switch (can->tx_seq) {
        case CAN_SEQ_IDLE:
            break;
        case CAN_SEQ_FIRST_FRAME: {
            if (!can->config.is_mailboxes_free()) {
                return;
            }
            uint8_t payload[8];
            uint32_t len = 0;
            memcpy(payload, &can->tx_frame.start_of_frame, sizeof(can->tx_frame.start_of_frame));
            len += sizeof(can->tx_frame.start_of_frame);
            memcpy(payload + len, &can->tx_frame.total_data_length, sizeof(can->tx_frame.total_data_length));
            len += sizeof(can->tx_frame.total_data_length);
            if (can->config.send_data(can->tx_frame.id, payload, len) == 0) {
                can->tx_seq = CAN_SEQ_MIDLE_FRAME;
                can->tx_frame.segment = 0;
                can->tx_frame.remaining_data = can->tx_frame.total_data_length;
            }
            break;
        }
        case CAN_SEQ_MIDLE_FRAME: {
            if (!can->config.is_mailboxes_free()) {
                return;
            }
            uint8_t payload[8];
            uint32_t len = 0;
            uint16_t data_len = (can->tx_frame.remaining_data > 6)? 6 : can->tx_frame.remaining_data;
            uint16_t data_offset = can->tx_frame.segment * 6;

            memcpy(payload, &can->tx_frame.segment, sizeof(can->tx_frame.segment));
            len += sizeof(can->tx_frame.segment);
            memcpy(payload + len, can->tx_frame.data + data_offset, data_len);
            len += data_len;
            if (can->config.send_data(can->tx_frame.id, payload, len) == 0) {
                can->tx_frame.segment++;
                can->tx_frame.remaining_data -= data_len;
                if (can->tx_frame.remaining_data == 0) {
                    can->tx_seq = CAN_SEQ_LAST_FRAME;
                }
            }
            break;
        }
        case CAN_SEQ_LAST_FRAME: {
            if (!can->config.is_mailboxes_free()) {
                return;
            }
            uint8_t payload[8];
            uint32_t len = 0;
            memcpy(payload, &can->tx_frame.end_of_frame, sizeof(can->tx_frame.end_of_frame));
            len += sizeof(can->tx_frame.end_of_frame);
            if (can->config.send_data(can->tx_frame.id, payload, len) == 0) {
                can->tx_seq = CAN_SEQ_IDLE;
            }
            break;
        }
    }
#else
    switch (can->tx_seq) {
        case CAN_SEQ_IDLE:
            break;
        case CAN_SEQ_FIRST_FRAME: {
            if (can->config.is_mailboxes_free()) {
                if (can->config.send_data(can->tx_frame.id, can->tx_frame.data, can->tx_frame.total_data_length) == 0) {
                    can->tx_seq = CAN_SEQ_IDLE;
                }
            }
            break;
        }
    }
#endif
}

int can_motor_recv_frame_update(can_protocol_t *can) {
#if 1
    if (can->incomming_frame_flag) {
        can->incomming_frame_flag = 0;
        _Bool is_data_frame = 0;

        if (can->frame_recv_length >= 4) {
            uint32_t header = 0;
            int frame_idx = 0;
            memcpy(&header, can->frame_recv, sizeof(header));
            if (header == 0xDEADBEEF) {
                if (can->frame_recv_length >= 6) {
                    frame_idx += sizeof(header);
                    memcpy(&can->rx_frame.total_data_length, &can->frame_recv[frame_idx], sizeof(can->rx_frame.total_data_length));
                    // if (can->rx_frame.total_data_length == 0) return -1;
                    can->rx_frame.remaining_data = can->rx_frame.total_data_length;
                    can->rx_frame.segment = 0;
                }
                else {
                    return -1;
                }
            }
            else if (header == 0x5F0C5F0C) {
                return 0;
            }
            else {
                is_data_frame = 1;
            }
        }
        else {
            is_data_frame = 1;
        }

        if (is_data_frame) {
            if (can->frame_recv_length > 2) {
                uint16_t current_segment = 0;
                int frame_idx = 0;
                memcpy(&current_segment, can->frame_recv, sizeof(current_segment));
                frame_idx += sizeof(current_segment);

                if (current_segment == can->rx_frame.segment) {
                    can->rx_frame.segment++;
                    uint16_t expected_data_len = (can->rx_frame.remaining_data > 6)? 6 : can->rx_frame.remaining_data;

                    uint16_t data_offset = can->rx_frame.total_data_length - can->rx_frame.remaining_data;
                    memcpy(can->rx_frame.data + data_offset, &can->frame_recv[frame_idx], expected_data_len);

                    can->rx_frame.remaining_data -= expected_data_len;
                    if (can->rx_frame.remaining_data == 0) {
                    }
                }
            }
        }
    }
    return -1;
#else
    if (can->incomming_frame_flag) {
        can->incomming_frame_flag = 0;
        can->rx_frame.data = &can->frame_recv[0];
        can->rx_frame.total_data_length = can->frame_recv_length;
        return 0;
    }
    return -1;
#endif
}

/******************************************************************************/
