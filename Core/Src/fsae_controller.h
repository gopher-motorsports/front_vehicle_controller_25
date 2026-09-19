/*
 * fsae_controller.h - the driving controller that runs a flashed policy.
 *
 * Call pattern (every control period, 20 ms, matching training):
 *
 *     fsae_policy_init(&pol, slot_address, slot_size);    // once per boot / mission
 *     fsae_ctrl_init(&ctl, &pol, &vehicle_cfg);
 *     fsae_ctrl_start(&ctl, now_ms);                      // on GO (staged start)
 *     loop:
 *         if (new perception frame) fsae_ctrl_perception(&ctl, &frame);
 *         fsae_ctrl_step(&ctl, &inputs, &out);
 *         if (out.perception_stale) -> emergency brake (the controller does not)
 *         else send out.steer_rad, out.drive_torque_nm, out.brake_* to actuators
 *
 * What the controller does, so the car sees what the policy saw in training:
 *   - estimates vx (undriven wheel speeds blended with the IMU), yaw rate (gyro)
 *     and vy (0, or a value supplied by the perception computer);
 *   - dead-reckons the pose and moves the latest cone list from its capture time
 *     to now, so perception latency and a 10-20 Hz frame rate are absorbed;
 *   - keeps only cones inside the sensor view the policy was trained with, and
 *     scales them the same way;
 *   - tracks the mission (timing lines crossed -> progress, phase flags,
 *     next_turn) from the event sequence stored in the model file;
 *   - applies launch assist, the action mode's rate limits, and a speed cap;
 *   - converts the action to a front wheel angle and wheel torques.
 *
 * Safety (the DSMS / EBS / remote-stop state machine, actuator limits,
 * plausibility checks) belongs to the surrounding firmware, not here.
 */
#ifndef FSAE_CONTROLLER_H
#define FSAE_CONTROLLER_H

#include <stdint.h>
#include "fsae_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FSAE_MAX_CONES 64
#define FSAE_HISTORY 64                 /* pose history for latency compensation */

enum { FSAE_CONE_BLUE = 0, FSAE_CONE_YELLOW = 1, FSAE_CONE_ORANGE = 2, FSAE_CONE_ORANGE_LARGE = 3 };

/* Properties of the real car and its wiring. Not part of the model. */
typedef struct {
    float    wheel_radius_m;            /* rolling radius for wheel-speed velocity */
    uint8_t  speed_wheels_mask;         /* wheels averaged for vx: bit0 FL, bit1 FR, bit2 RL, bit3 RR (undriven ones) */
    float    vx_blend;                  /* 0..1 weight on the IMU prediction (e.g. 0.9) */
    float    fov_margin_rad;            /* extra view allowed for detection noise: about 3x the
                                           bearing noise of your perception (e.g. 5 deg = 0.087) */
    float    range_margin_frac;         /* extra range allowed, about 3x the relative range noise (e.g. 0.05) */
    uint32_t perception_timeout_ms;     /* older frames -> perception_stale (e.g. 200) */
    float    max_speed_mps;             /* no throttle above this; 0 = no cap */
} fsae_vehicle_cfg_t;

typedef struct {
    float   x, y;                       /* m, vehicle frame at capture: x forward, y left, origin at the CG */
    uint8_t cls;                        /* FSAE_CONE_* */
} fsae_cone_t;

typedef struct {
    uint32_t    capture_ms;             /* controller clock at sensor capture */
    uint16_t    n;
    fsae_cone_t cones[FSAE_MAX_CONES];
} fsae_perception_t;

typedef struct {
    uint32_t now_ms;
    float    gyro_z;                    /* rad/s, counter-clockwise positive */
    float    accel_x;                   /* m/s^2, forward specific force */
    float    wheel_speed[4];            /* rad/s: FL, FR, RL, RR */
    uint8_t  crossings;                 /* timing lines crossed so far (from perception) */
    float    vy_ext;                    /* m/s from the perception computer, or NAN */
    float    vx_ext;                    /* m/s override (testing), or NAN */
} fsae_inputs_t;

typedef struct {
    float    action[2];                 /* policy output, -1..1 */
    float    applied[2];                /* after launch assist, rate limits, speed cap */
    float    steer_rad;                 /* front wheel angle, positive left */
    float    drive_torque_nm;           /* per driven wheel, at the wheel */
    float    total_drive_torque_nm;     /* drive_torque_nm x the model's driven wheels */
    float    brake_torque_front_nm;     /* front axle total */
    float    brake_torque_rear_nm;      /* rear axle total */
    float    vx_est, vy_est;
    uint16_t n_cones_used;
    uint8_t  mission_finished;          /* past the last timing line: stop the car */
    uint8_t  perception_stale;          /* no frame within perception_timeout_ms (counted from
                                           start until the first frame): caller must brake */
    uint8_t  launch_assist;             /* throttle held by launch assist */
    float    obs[FSAE_MAX_WIDTH];       /* the observation given to the policy */
} fsae_outputs_t;

typedef struct {
    uint32_t ms;
    float    x, y, psi;
} fsae_pose_t;

typedef struct {
    const fsae_policy_t *pol;
    fsae_vehicle_cfg_t   cfg;
    fsae_pose_t          hist[FSAE_HISTORY];
    uint32_t             hist_n, hist_head;
    fsae_pose_t          pose;
    uint32_t             last_ms;
    uint32_t             start_ms;
    uint8_t              have_last;
    float                vx_est;
    float                prev[2];
    uint32_t             next_cross;
    uint8_t              assisting;
    uint8_t              have_frame;
    uint32_t             frame_ms;
    uint8_t              have_pending;
    fsae_perception_t    pending;       /* latest frame, placed at the next step */
    float                vy_last, r_last;
    uint16_t             n_cones;
    float                cone_x[FSAE_MAX_CONES], cone_y[FSAE_MAX_CONES];  /* odometry frame */
    uint8_t              cone_cls[FSAE_MAX_CONES];
} fsae_ctrl_t;

void fsae_ctrl_init(fsae_ctrl_t *c, const fsae_policy_t *pol, const fsae_vehicle_cfg_t *cfg);
void fsae_ctrl_start(fsae_ctrl_t *c, uint32_t now_ms);
/* Hand over a new cone list. It is placed in the odometry frame at the next
 * fsae_ctrl_step(), once the pose has been advanced to that step's time, so a
 * frame captured "now" lines up with the car's pose now. */
void fsae_ctrl_perception(fsae_ctrl_t *c, const fsae_perception_t *frame);
void fsae_ctrl_step(fsae_ctrl_t *c, const fsae_inputs_t *in, fsae_outputs_t *out);
uint32_t fsae_ctrl_sizeof(void);        /* for bindings */

#ifdef __cplusplus
}
#endif
#endif /* FSAE_CONTROLLER_H */
