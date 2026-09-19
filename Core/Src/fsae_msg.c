/* fsae_msg.c - see fsae_msg.h */
#include "fsae_msg.h"

#include <math.h>

uint16_t fsae_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int k = 0; k < 8; k++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t i16le(const uint8_t *p) { return (int16_t)u16le(p); }

fsae_msg_status_t fsae_msg_decode(const uint8_t *buf, uint16_t len, uint32_t rx_ms,
                                  fsae_perception_t *frame, uint8_t *crossings, float *vy)
{
    if (len < 10) return FSAE_MSG_ERR_LEN;
    if (buf[0] != FSAE_MSG_VERSION) return FSAE_MSG_ERR_VERSION;
    uint8_t n = buf[1];
    if (n > FSAE_MAX_CONES) return FSAE_MSG_ERR_COUNT;
    if (len != 10u + 6u * n) return FSAE_MSG_ERR_LEN;
    if (fsae_crc16(buf, (uint16_t)(len - 2)) != u16le(buf + len - 2)) return FSAE_MSG_ERR_CRC;

    uint16_t age = u16le(buf + 4);
    frame->capture_ms = rx_ms > age ? rx_ms - age : 0;
    frame->n = n;
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *c = buf + 8 + 6u * i;
        frame->cones[i].x = (float)i16le(c) * 0.01f;
        frame->cones[i].y = (float)i16le(c + 2) * 0.01f;
        frame->cones[i].cls = c[4] > FSAE_CONE_ORANGE_LARGE ? FSAE_CONE_ORANGE : c[4];
    }
    *crossings = buf[2];
    *vy = (buf[3] & 1u) ? (float)i16le(buf + 6) * 0.01f : NAN;
    return FSAE_MSG_OK;
}

enum { ST_SYNC1 = 0, ST_SYNC2, ST_LEN1, ST_LEN2, ST_BODY };

void fsae_msg_stream_init(fsae_msg_stream_t *s)
{
    s->len = 0;
    s->pos = 0;
    s->state = ST_SYNC1;
    s->dropped = 0;
}

uint16_t fsae_msg_stream_push(fsae_msg_stream_t *s, uint8_t byte)
{
    switch (s->state) {
    case ST_SYNC1:
        if (byte == 0xA5) s->state = ST_SYNC2;
        return 0;
    case ST_SYNC2:
        s->state = (byte == 0x5A) ? ST_LEN1 : (byte == 0xA5 ? ST_SYNC2 : ST_SYNC1);
        return 0;
    case ST_LEN1:
        s->len = byte;
        s->state = ST_LEN2;
        return 0;
    case ST_LEN2:
        s->len |= (uint16_t)(byte << 8);
        if (s->len < 10 || s->len > FSAE_MSG_MAX_LEN) {
            s->dropped++;
            s->state = ST_SYNC1;
            return 0;
        }
        s->pos = 0;
        s->state = ST_BODY;
        return 0;
    default:
        s->buf[s->pos++] = byte;
        if (s->pos < s->len) return 0;
        s->state = ST_SYNC1;
        return s->len;
    }
}
