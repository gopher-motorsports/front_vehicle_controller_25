/* fsae_controller.c - see fsae_controller.h */
#include "fsae_controller.h"

#include <math.h>
#include <string.h>

#define CONTROL_DT_DEFAULT 0.02f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void fsae_ctrl_init(fsae_ctrl_t *c, const fsae_policy_t *pol, const fsae_vehicle_cfg_t *cfg)
{
    memset(c, 0, sizeof(*c));
    c->pol = pol;
    c->cfg = *cfg;
}

void fsae_ctrl_start(fsae_ctrl_t *c, uint32_t now_ms)
{
    const fsae_policy_t *pol = c->pol;
    fsae_vehicle_cfg_t cfg = c->cfg;
    fsae_ctrl_init(c, pol, &cfg);
    c->last_ms = now_ms;
    c->start_ms = now_ms;
    c->have_last = 1;
    c->assisting = pol->h->launch_assist_speed > 0.0f;
    c->hist[0] = c->pose;
    c->hist[0].ms = now_ms;
    c->hist_n = 1;
    c->hist_head = 1 % FSAE_HISTORY;
}

static void push_pose(fsae_ctrl_t *c)
{
    c->hist[c->hist_head] = c->pose;
    c->hist_head = (c->hist_head + 1) % FSAE_HISTORY;
    if (c->hist_n < FSAE_HISTORY) c->hist_n++;
}

/* The recorded pose closest in time to ms (the oldest if ms is older). */
static fsae_pose_t pose_at(const fsae_ctrl_t *c, uint32_t ms)
{
    fsae_pose_t best = c->pose;
    uint32_t best_d = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < c->hist_n; i++) {
        const fsae_pose_t *p = &c->hist[(c->hist_head + FSAE_HISTORY - 1 - i) % FSAE_HISTORY];
        uint32_t d = p->ms > ms ? p->ms - ms : ms - p->ms;
        if (d < best_d) { best_d = d; best = *p; }
    }
    return best;
}

void fsae_ctrl_perception(fsae_ctrl_t *c, const fsae_perception_t *frame)
{
    c->pending = *frame;
    c->have_pending = 1;
}

static void place_frame(fsae_ctrl_t *c, const fsae_perception_t *frame)
{
    fsae_pose_t p = pose_at(c, frame->capture_ms);
    float cs = cosf(p.psi), sn = sinf(p.psi);
    uint16_t n = frame->n > FSAE_MAX_CONES ? FSAE_MAX_CONES : frame->n;
    for (uint16_t i = 0; i < n; i++) {
        const fsae_cone_t *k = &frame->cones[i];
        c->cone_x[i] = p.x + cs * k->x - sn * k->y;
        c->cone_y[i] = p.y + sn * k->x + cs * k->y;
        c->cone_cls[i] = k->cls > FSAE_CONE_ORANGE_LARGE ? FSAE_CONE_ORANGE : k->cls;
    }
    c->n_cones = n;
    c->frame_ms = frame->capture_ms;
    c->have_frame = 1;
}

void fsae_ctrl_step(fsae_ctrl_t *c, const fsae_inputs_t *in, fsae_outputs_t *out)
{
    const fsae_policy_header_t *h = c->pol->h;
    memset(out, 0, sizeof(*out));
    float dt = CONTROL_DT_DEFAULT;
    if (c->have_last && in->now_ms > c->last_ms) dt = (float)(in->now_ms - c->last_ms) * 1e-3f;
    c->last_ms = in->now_ms;
    c->have_last = 1;

    /* --- dead reckoning: advance the pose to now with the last estimates --- */
    c->pose.psi = wrap_pi(c->pose.psi + c->r_last * dt);
    {
        float cp = cosf(c->pose.psi), sp = sinf(c->pose.psi);
        c->pose.x += (c->vx_est * cp - c->vy_last * sp) * dt;
        c->pose.y += (c->vx_est * sp + c->vy_last * cp) * dt;
    }
    c->pose.ms = in->now_ms;
    push_pose(c);
    if (c->have_pending) {
        place_frame(c, &c->pending);
        c->have_pending = 0;
    }
    float cs = cosf(c->pose.psi), sn = sinf(c->pose.psi);

    /* --- ego state at now --- */
    float r = in->gyro_z;
    float wsum = 0.0f;
    int wn = 0;
    for (int i = 0; i < 4; i++)
        if (c->cfg.speed_wheels_mask & (1u << i)) { wsum += in->wheel_speed[i]; wn++; }
    float vx_wheel = wn ? wsum / (float)wn * c->cfg.wheel_radius_m : c->vx_est;
    if (isfinite(in->vx_ext)) {
        c->vx_est = in->vx_ext;
    } else {
        float pred = c->vx_est + in->accel_x * dt;
        c->vx_est = c->cfg.vx_blend * pred + (1.0f - c->cfg.vx_blend) * vx_wheel;
        if (c->vx_est < 0.0f && vx_wheel >= 0.0f) c->vx_est = 0.0f;
    }
    float vx = c->vx_est;
    float vy = isfinite(in->vy_ext) ? in->vy_ext : 0.0f;
    c->vy_last = vy;
    c->r_last = r;

    /* --- mission --- */
    if (in->crossings > c->next_cross)
        c->next_cross = in->crossings > h->n_crossings ? h->n_crossings : in->crossings;
    uint8_t finished = c->next_cross >= h->n_crossings;

    /* --- observation (same order and scaling as env.py) --- */
    float *o = out->obs;
    o[0] = vx;
    o[1] = vy;
    o[2] = r;
    o[3] = c->prev[0];
    o[4] = c->prev[1];
    o[5] = (float)c->next_cross / (float)h->n_crossings;
    o[6] = c->next_cross == 0 ? 1.0f : 0.0f;
    o[7] = finished ? 1.0f : 0.0f;
    o[8] = finished ? 0.0f : h->crossing_turns[c->next_cross];

    /* cones: move to the current vehicle frame, filter, sort nearest first */
    float rng[FSAE_MAX_CONES], brg[FSAE_MAX_CONES];
    uint8_t idx[FSAE_MAX_CONES];
    uint16_t m = 0;
    if (c->have_frame) {
        float max_r = h->filter_range_m * (1.0f + c->cfg.range_margin_frac);
        float max_b = h->filter_half_fov_rad + c->cfg.fov_margin_rad;
        for (uint16_t i = 0; i < c->n_cones; i++) {
            float dx = c->cone_x[i] - c->pose.x, dy = c->cone_y[i] - c->pose.y;
            float bx = cs * dx + sn * dy, by = -sn * dx + cs * dy;
            float rr = sqrtf(bx * bx + by * by);
            float bb = atan2f(by, bx);
            if (rr > max_r || fabsf(bb) > max_b) continue;
            rng[i] = rr;
            brg[i] = bb;
            /* insertion sort by range. Ranges within the perception message's
             * resolution (1 cm per axis, so up to ~1.5 cm) keep the order the
             * perception computer sent, which it sorted on unrounded values;
             * left/right cone pairs are often this close. */
            uint16_t j = m++;
            while (j > 0 && rng[idx[j - 1]] > rr + 0.015f) { idx[j] = idx[j - 1]; j--; }
            idx[j] = (uint8_t)i;
        }
    }
    uint32_t k_max = h->top_k;
    for (uint32_t s = 0; s < k_max; s++) {
        float *slot = o + 9 + s * h->slot_size;
        for (uint32_t q = 0; q < h->slot_size; q++) slot[q] = 0.0f;
        if (s >= m) continue;
        uint8_t i = idx[s];
        slot[0] = clampf(rng[i] / h->range_scale_m, 0.0f, 1.5f);
        slot[1] = clampf(brg[i] / h->bearing_scale_rad, -1.5f, 1.5f);
        slot[2 + c->cone_cls[i]] = 1.0f;
        slot[6] = 1.0f;
    }
    out->n_cones_used = m < k_max ? m : (uint16_t)k_max;
    {
        uint32_t since = c->have_frame ? c->frame_ms : c->start_ms;
        out->perception_stale = (in->now_ms - since > c->cfg.perception_timeout_ms) ? 1 : 0;
    }

    /* --- policy --- */
    fsae_policy_forward(c->pol, o, out->action);

    /* --- applied command --- */
    float a[2] = { out->action[0], out->action[1] };
    if (h->action_mode == FSAE_MODE_TARGET) {
        float lim[2] = { h->steer_rate * dt, h->pedal_rate * dt };
        for (int i = 0; i < 2; i++) a[i] = c->prev[i] + clampf(a[i] - c->prev[i], -lim[i], lim[i]);
    } else if (h->action_mode == FSAE_MODE_DELTA) {
        float rate[2] = { h->steer_rate, h->pedal_rate };
        for (int i = 0; i < 2; i++) a[i] = c->prev[i] + a[i] * rate[i] * dt;
    }
    a[0] = clampf(a[0], -1.0f, 1.0f);
    a[1] = clampf(a[1], -1.0f, 1.0f);
    float speed = sqrtf(vx * vx + vy * vy);
    if (c->assisting && speed >= h->launch_assist_speed) c->assisting = 0;
    if (c->assisting) a[1] = 1.0f;
    if (c->cfg.max_speed_mps > 0.0f && speed > c->cfg.max_speed_mps && a[1] > 0.0f) a[1] = 0.0f;
    c->prev[0] = a[0];
    c->prev[1] = a[1];
    out->applied[0] = a[0];
    out->applied[1] = a[1];
    out->launch_assist = c->assisting;

    /* --- physical commands (the scaling the policy was trained with) --- */
    out->steer_rad = a[0] * h->max_steer_rad;
    if (a[1] >= 0.0f) {
        out->drive_torque_nm = a[1] * h->max_motor_torque;
        out->total_drive_torque_nm = out->drive_torque_nm * (float)h->driven_wheels;
    } else {
        float total = -a[1] * h->max_brake_torque;
        out->brake_torque_front_nm = total * h->front_brake_bias;
        out->brake_torque_rear_nm = total * (1.0f - h->front_brake_bias);
    }
    out->vx_est = vx;
    out->vy_est = vy;
    out->mission_finished = finished;
}

uint32_t fsae_ctrl_sizeof(void) { return (uint32_t)sizeof(fsae_ctrl_t); }
