/*
 * fsae_policy.h - load a .fsaepol model file and run the policy network.
 *
 * The file is used in place (no copies): keep it in memory-mapped flash or a
 * 4-byte aligned RAM buffer. fsae_policy_init() checks the magic, format
 * version, size, CRC and layer shapes before anything uses it.
 *
 * Plain C99, no heap, no floating-point library beyond tanh().
 */
#ifndef FSAE_POLICY_H
#define FSAE_POLICY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSAEPOL_FORMAT_VERSION 1u
#define FSAE_MAX_LAYERS 8
#define FSAE_MAX_WIDTH 256
#define FSAE_MAX_CROSSINGS 8

enum { FSAE_EVENT_SKIDPAD = 0, FSAE_EVENT_ACCELERATION = 1, FSAE_EVENT_AUTOCROSS = 2 };
enum { FSAE_ACT_NONE = 0, FSAE_ACT_TANH = 1, FSAE_ACT_RELU = 2 };
enum { FSAE_MODE_ABSOLUTE = 0, FSAE_MODE_TARGET = 1, FSAE_MODE_DELTA = 2 };
#define FSAE_FLAG_SQUASH 1u

/* Mirrors HEADER_FMT in fsaepol.py. Every field is 4-byte aligned. */
typedef struct {
    char     magic[8];              /* "FSAEPOL1" */
    uint32_t format_version;
    uint32_t total_size;            /* bytes, including the trailing CRC */
    uint32_t event;                 /* FSAE_EVENT_* */
    uint32_t obs_dim;
    uint32_t act_dim;
    uint32_t n_layers;
    uint32_t flags;                 /* FSAE_FLAG_* */
    uint32_t top_k;                 /* cone slots in the observation */
    uint32_t slot_size;             /* values per cone slot (7) */
    float    range_scale_m;         /* range_norm = range / this */
    float    bearing_scale_rad;     /* bearing_norm = bearing / this */
    float    filter_range_m;        /* cones reported only within this range ... */
    float    filter_half_fov_rad;   /* ... and this half field of view */
    float    clip_obs;              /* normalised observation clip */
    float    norm_eps;
    float    max_steer_rad;         /* action +1 = this front wheel angle */
    float    max_motor_torque;      /* action +1 = this torque per driven wheel (N m) */
    float    max_brake_torque;      /* action -1 = this total brake torque (N m) */
    float    front_brake_bias;
    float    launch_assist_speed;   /* m/s; 0 = no launch assist */
    uint32_t n_crossings;           /* timing lines in the run */
    float    crossing_turns[FSAE_MAX_CROSSINGS]; /* +1 left, -1 right, 0 straight */
    char     name[32];
    uint32_t created_unix;
    uint32_t action_mode;           /* FSAE_MODE_* */
    float    steer_rate;            /* full-scale units per second (target/delta) */
    float    pedal_rate;
    uint32_t driven_wheels;         /* wheels max_motor_torque applies to (2 = RWD) */
} fsae_policy_header_t;

typedef struct {
    uint32_t in, out, act;
    const float *W;                 /* out x in, row-major */
    const float *b;
} fsae_layer_t;

typedef struct {
    const fsae_policy_header_t *h;
    const float *mean;              /* obs_dim */
    const float *std;               /* obs_dim, sqrt(var + eps) */
    fsae_layer_t layers[FSAE_MAX_LAYERS];
} fsae_policy_t;

typedef enum {
    FSAE_OK = 0,
    FSAE_ERR_ALIGN,                 /* blob not 4-byte aligned */
    FSAE_ERR_MAGIC,
    FSAE_ERR_VERSION,
    FSAE_ERR_SIZE,
    FSAE_ERR_CRC,
    FSAE_ERR_SHAPE
} fsae_status_t;

uint32_t fsae_crc32(const uint8_t *data, uint32_t len);
uint32_t fsae_policy_sizeof(void);      /* for bindings */

/* Validate `blob` (at most max_len bytes available) and index it into `p`. */
fsae_status_t fsae_policy_init(fsae_policy_t *p, const uint8_t *blob, uint32_t max_len);

/* Normalise `obs` (obs_dim values) and run the network; writes act_dim values
 * clipped to [-1, 1]. Not reentrant (uses static scratch buffers). */
void fsae_policy_forward(const fsae_policy_t *p, const float *obs, float *action_out);

#ifdef __cplusplus
}
#endif
#endif /* FSAE_POLICY_H */
