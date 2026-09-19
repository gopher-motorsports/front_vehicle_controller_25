/*
 * fsae_msg.h - the perception message the onboard computer sends to the
 * vehicle controller: one cone list per message, transport-agnostic (UART,
 * Ethernet, or split across CAN FD frames).
 *
 * Layout, little-endian:
 *   u8  version (1)
 *   u8  n                 cones that follow (<= 64)
 *   u8  crossings         timing lines crossed so far in this run
 *   u8  flags             bit0: vy_cm_s is valid
 *   u16 age_ms            time from sensor capture to sending
 *   i16 vy_cm_s           lateral velocity estimate, cm/s
 *   n x { i16 x_cm, i16 y_cm, u8 cls, u8 reserved }   vehicle frame at capture,
 *                                                     origin at the CG, x forward, y left
 *   u16 crc16             CRC-16/CCITT-FALSE over everything before it
 *
 * Size: 10 + 6 n bytes (394 at most).
 *
 * Over a byte stream (UART), each message is framed as
 *   0xA5 0x5A, u16 length (little-endian), message
 * and fsae_msg_stream_push() reassembles it byte by byte.
 *
 * age_ms avoids synchronising clocks: the controller takes capture time as
 * its receive time minus age_ms (plus any known transport delay).
 */
#ifndef FSAE_MSG_H
#define FSAE_MSG_H

#include <stdint.h>
#include "fsae_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FSAE_MSG_VERSION 1u
#define FSAE_MSG_MAX_LEN (10u + 6u * FSAE_MAX_CONES)

typedef enum {
    FSAE_MSG_OK = 0,
    FSAE_MSG_ERR_LEN,
    FSAE_MSG_ERR_VERSION,
    FSAE_MSG_ERR_CRC,
    FSAE_MSG_ERR_COUNT
} fsae_msg_status_t;

uint16_t fsae_crc16(const uint8_t *data, uint16_t len);

/* Byte-stream reassembly for UART links. */
typedef struct {
    uint8_t  buf[FSAE_MSG_MAX_LEN];
    uint16_t len;           /* payload length from the frame header */
    uint16_t pos;
    uint8_t  state;
    uint32_t dropped;       /* frames discarded for a bad length */
} fsae_msg_stream_t;

void fsae_msg_stream_init(fsae_msg_stream_t *s);
/* Feed one received byte. Returns the payload length when a complete frame is
 * in s->buf (pass s->buf and that length to fsae_msg_decode), otherwise 0. */
uint16_t fsae_msg_stream_push(fsae_msg_stream_t *s, uint8_t byte);

/* Decode one message received at controller time rx_ms. On success fills
 * `frame` (for fsae_ctrl_perception) and the crossing count and vy (NAN when
 * not supplied) for the next fsae_inputs_t. */
fsae_msg_status_t fsae_msg_decode(const uint8_t *buf, uint16_t len, uint32_t rx_ms,
                                  fsae_perception_t *frame, uint8_t *crossings, float *vy);

#ifdef __cplusplus
}
#endif
#endif /* FSAE_MSG_H */
