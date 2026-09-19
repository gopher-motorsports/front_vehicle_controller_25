/* fsae_policy.c - see fsae_policy.h */
#include "fsae_policy.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(fsae_policy_header_t) == 176, "header layout must match fsaepol.py");
_Static_assert(sizeof(float) == 4, "float must be 32-bit");

uint32_t fsae_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

static uint32_t read_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
    return v;
}

fsae_status_t fsae_policy_init(fsae_policy_t *p, const uint8_t *blob, uint32_t max_len)
{
    memset(p, 0, sizeof(*p));
    if (((uintptr_t)blob & 3u) != 0) return FSAE_ERR_ALIGN;
    if (max_len < sizeof(fsae_policy_header_t) + 4) return FSAE_ERR_SIZE;
    const fsae_policy_header_t *h = (const fsae_policy_header_t *)(const void *)blob;
    if (memcmp(h->magic, "FSAEPOL1", 8) != 0) return FSAE_ERR_MAGIC;
    if (h->format_version != FSAEPOL_FORMAT_VERSION) return FSAE_ERR_VERSION;
    if (h->total_size > max_len || h->total_size % 4 != 0) return FSAE_ERR_SIZE;
    if (fsae_crc32(blob, h->total_size - 4) != read_u32(blob + h->total_size - 4))
        return FSAE_ERR_CRC;
    if (h->obs_dim == 0 || h->obs_dim > FSAE_MAX_WIDTH || h->n_layers == 0 ||
        h->n_layers > FSAE_MAX_LAYERS || h->act_dim != 2 ||
        h->n_crossings == 0 || h->n_crossings > FSAE_MAX_CROSSINGS ||
        h->obs_dim != 9 + h->top_k * h->slot_size)
        return FSAE_ERR_SHAPE;

    uint32_t off = sizeof(fsae_policy_header_t);
    const uint32_t end = h->total_size - 4;
    const float *f = (const float *)(const void *)blob;
    p->h = h;
    p->mean = f + off / 4; off += 4 * h->obs_dim;
    p->std = f + off / 4;  off += 4 * h->obs_dim;
    uint32_t width = h->obs_dim;
    for (uint32_t i = 0; i < h->n_layers; i++) {
        if (off + 12 > end) return FSAE_ERR_SIZE;
        fsae_layer_t *L = &p->layers[i];
        L->in = read_u32(blob + off);
        L->out = read_u32(blob + off + 4);
        L->act = read_u32(blob + off + 8);
        off += 12;
        if (L->in != width || L->out == 0 || L->out > FSAE_MAX_WIDTH || L->act > FSAE_ACT_RELU)
            return FSAE_ERR_SHAPE;
        uint32_t n = 4 * (L->in * L->out + L->out);
        if (off + n > end) return FSAE_ERR_SIZE;
        L->W = f + off / 4;
        L->b = f + (off + 4 * L->in * L->out) / 4;
        off += n;
        width = L->out;
    }
    if (width != h->act_dim || off != end) return FSAE_ERR_SHAPE;
    return FSAE_OK;
}

void fsae_policy_forward(const fsae_policy_t *p, const float *obs, float *action_out)
{
    static float buf_a[FSAE_MAX_WIDTH], buf_b[FSAE_MAX_WIDTH];
    const fsae_policy_header_t *h = p->h;
    float *x = buf_a, *y = buf_b;
    for (uint32_t i = 0; i < h->obs_dim; i++) {
        float v = (obs[i] - p->mean[i]) / p->std[i];
        if (v > h->clip_obs) v = h->clip_obs;
        if (v < -h->clip_obs) v = -h->clip_obs;
        x[i] = v;
    }
    for (uint32_t l = 0; l < h->n_layers; l++) {
        const fsae_layer_t *L = &p->layers[l];
        for (uint32_t r = 0; r < L->out; r++) {
            const float *w = L->W + r * L->in;
            float s = L->b[r];
            for (uint32_t c = 0; c < L->in; c++) s += w[c] * x[c];
            if (L->act == FSAE_ACT_TANH) s = tanhf(s);
            else if (L->act == FSAE_ACT_RELU && s < 0.0f) s = 0.0f;
            y[r] = s;
        }
        float *t = x; x = y; y = t;
    }
    for (uint32_t i = 0; i < h->act_dim; i++) {
        float v = x[i];
        if (h->flags & FSAE_FLAG_SQUASH) v = tanhf(v);
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        action_out[i] = v;
    }
}

uint32_t fsae_policy_sizeof(void) { return (uint32_t)sizeof(fsae_policy_t); }
