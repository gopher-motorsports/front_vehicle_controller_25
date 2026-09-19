/* gopher_ds.c - see gopher_ds.h */
#include "gopher_ds.h"

#include <math.h>
#include <string.h>

#include "GopherCAN.h"
#include "gopher_sense.h"
#include "fsae_msg.h"
#include "fsae_policy.h"

#define DEG2RAD (3.14159265f / 180.0f)
#define G_MPS2  9.80665f
#define STATUS_PERIOD_TICKS 5          /* telemetry every 100 ms */

static gds_config_t      s_cfg;
static fsae_policy_t     s_policy;
static uint8_t           s_model_ok;
static fsae_ctrl_t       s_ctrl;
static fsae_msg_stream_t s_stream;
static gds_state_t       s_state = GDS_IDLE;
static gds_fault_t       s_fault = GDS_FAULT_NONE;
static gds_command_t     s_cmd;
static fsae_outputs_t    s_out;
static uint8_t           s_crossings;
static float             s_vy_msg = NAN;
static uint32_t          s_last_frame_ms;
static uint32_t          s_ticks;

/* ------------------------------------------------------------ inputs --- */
__weak uint32_t gds_millis(void) { return HAL_GetTick(); }

__weak int gds_read_ins(float *gyro_z_degps, float *vel_x, float *vel_y, uint16_t *status,
                        uint32_t *t_ms)
{
    *gyro_z_degps = fvcGyroBodyZ.data;
    *vel_x = fvcVelBodyX.data;
    *vel_y = fvcVelBodyY.data;
    *status = fvcINS_status.data;
    /* oldest of the three groups, so a missing group shows as stale */
    uint32_t t = fvcGyroBodyZ.info.last_rx;
    if (fvcVelBodyX.info.last_rx < t) t = fvcVelBodyX.info.last_rx;
    *t_ms = t;
    return t != 0;
}

__weak int gds_read_accel_x(float *accel_g, uint32_t *t_ms)
{
    *accel_g = longitudinalAccel_G.data;
    *t_ms = longitudinalAccel_G.info.last_rx;
    return *t_ms != 0;
}

__weak int gds_read_wheel_speeds(float mps[4], uint32_t *t_ms)
{
    mps[0] = wheelSpeedFrontLeft_mph.data * 0.44704;
    mps[1] = wheelSpeedFrontRight_mph.data * 0.44704;
    mps[2] = wheelSpeedRearLeft_mph.data * 0.44704;
    mps[3] = wheelSpeedRearRight_mph.data *0.44704;
    uint32_t t = wheelSpeedFrontLeft_mph.info.last_rx;
    if (wheelSpeedRearLeft_mph.info.last_rx < t) t = wheelSpeedRearLeft_mph.info.last_rx;
    *t_ms = t;
    return t != 0;
}

/* ------------------------------------------------------------ setup ---- */
void gds_default_config(gds_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->model_addr = (const uint8_t *)0x08060000u;   /* F446 sector 7 */
    cfg->model_size = 0x20000u;
    cfg->vehicle.wheel_radius_m = 0.2006f;            /* rolling radius at running load: measure */
    cfg->vehicle.speed_wheels_mask = 0x0F;            /* all four (AWD: none are undriven) */
    cfg->vehicle.vx_blend = 0.95f;
    cfg->vehicle.fov_margin_rad = 0.087f;
    cfg->vehicle.range_margin_frac = 0.05f;
    cfg->vehicle.perception_timeout_ms = 200;
    cfg->vehicle.max_speed_mps = 5.0f;                /* raise step by step during testing */
    cfg->fvc_y_sign = -1.0f;
    cfg->fvc_z_sign = -1.0f;
    cfg->accel_x_sign = 1.0f;
    cfg->use_ins_velocity = 1;
    cfg->ins_status_mask = 0;
    cfg->ins_timeout_ms = 50;
    cfg->wheels_timeout_ms = 50;
}

static void set_fault(gds_fault_t f)
{
    s_fault = f;
    s_state = GDS_FAULT;
    memset(&s_cmd, 0, sizeof(s_cmd));
}

int gds_init(const gds_config_t *cfg)
{
    s_cfg = *cfg;
    fsae_msg_stream_init(&s_stream);
    s_model_ok = fsae_policy_init(&s_policy, cfg->model_addr, cfg->model_size) == FSAE_OK;
    s_state = GDS_IDLE;
    s_fault = s_model_ok ? GDS_FAULT_NONE : GDS_FAULT_MODEL;
    memset(&s_cmd, 0, sizeof(s_cmd));
    return s_model_ok ? 0 : -1;
}

int gds_select_mission(uint8_t event)
{
    if (!s_model_ok) { set_fault(GDS_FAULT_MODEL); return -1; }
    if (s_policy.h->event != event) { set_fault(GDS_FAULT_WRONG_EVENT); return -1; }
    fsae_ctrl_init(&s_ctrl, &s_policy, &s_cfg.vehicle);
    s_state = GDS_READY;
    s_fault = GDS_FAULT_NONE;
    return 0;
}

void gds_go(void)
{
    if (s_state != GDS_READY) return;
    s_crossings = 0;
    s_vy_msg = NAN;
    s_last_frame_ms = gds_millis();       /* perception gets one timeout to start */
    memset(&s_out, 0, sizeof(s_out));
    fsae_ctrl_start(&s_ctrl, gds_millis());
    s_state = GDS_DRIVING;
}

void gds_stop(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    if (s_state == GDS_DRIVING) s_state = GDS_READY;
}

const gds_command_t *gds_command(void) { return &s_cmd; }
gds_state_t gds_state(void) { return s_state; }
gds_fault_t gds_fault(void) { return s_fault; }
const fsae_policy_header_t *gds_model(void) { return s_model_ok ? s_policy.h : NULL; }
const fsae_outputs_t *gds_debug_outputs(void) { return &s_out; }
const fsae_ctrl_t *gds_debug_ctrl(void) { return &s_ctrl; }

/* ------------------------------------------------------------ perception */
void gds_uart_rx_byte(uint8_t byte)
{
    uint16_t n = fsae_msg_stream_push(&s_stream, byte);
    if (!n) return;
    fsae_perception_t frame;
    uint8_t crossings;
    float vy;
    if (fsae_msg_decode(s_stream.buf, n, gds_millis(), &frame, &crossings, &vy) != FSAE_MSG_OK)
        return;
    if (s_state != GDS_DRIVING) return;
    fsae_ctrl_perception(&s_ctrl, &frame);
    if (crossings > s_crossings) s_crossings = crossings;
    s_vy_msg = vy;
    s_last_frame_ms = gds_millis();
}

/* ------------------------------------------------------------ telemetry */
static void publish(void)
{
    update_and_queue_param_float(&dsSteerCmd_deg, s_cmd.steer_deg);
    update_and_queue_param_float(&dsDriveTorqueCmd_Nm, s_cmd.total_drive_torque_nm);
    update_and_queue_param_float(&dsBrakeTorqueFrontCmd_Nm, s_cmd.brake_front_nm);
    update_and_queue_param_float(&dsBrakeTorqueRearCmd_Nm, s_cmd.brake_rear_nm);
    if (++s_ticks % STATUS_PERIOD_TICKS) return;
    uint8_t flags = (uint8_t)((s_out.perception_stale ? 1u : 0u) |
                              (s_out.launch_assist ? 2u : 0u) |
                              (s_out.mission_finished ? 4u : 0u) |
                              (s_cmd.valid ? 8u : 0u));
    update_and_queue_param_u8(&dsState_state, (uint8_t)s_state);
    update_and_queue_param_u8(&dsFault_state, (uint8_t)s_fault);
    update_and_queue_param_u8(&dsCrossings_state, s_crossings);
    update_and_queue_param_u8(&dsFlags_state, flags);
    update_and_queue_param_u8(&dsConesUsed_state, (uint8_t)s_out.n_cones_used);
    update_and_queue_param_u8(&dsModelEvent_state, s_model_ok ? (uint8_t)s_policy.h->event : 0xFF);
    update_and_queue_param_u16(&dsPerceptionAge_ms,
                               (uint16_t)((gds_millis() - s_last_frame_ms) > 65535u
                                              ? 65535u : (gds_millis() - s_last_frame_ms)));
    update_and_queue_param_float(&dsVxEst_mps, s_out.vx_est);
    update_and_queue_param_float(&dsVyEst_mps, s_out.vy_est);
    update_and_queue_param_float(&dsYawRate_degps, s_out.obs[2] / DEG2RAD);
    update_and_queue_param_u32(&dsModelCreated_unix, s_model_ok ? s_policy.h->created_unix : 0u);
}

/* ------------------------------------------------------------ control -- */
void gds_tick(void)
{
    uint32_t now = gds_millis();
    if (s_state != GDS_DRIVING) {
        s_cmd.valid = 0;
        publish();
        return;
    }

    float gyro_degps, vel_x, vel_y, accel_g, wheels[4];
    uint16_t ins_status;
    uint32_t t_ins, t_acc, t_wh;
    if (!gds_read_ins(&gyro_degps, &vel_x, &vel_y, &ins_status, &t_ins) ||
        now - t_ins > s_cfg.ins_timeout_ms) { set_fault(GDS_FAULT_INS_STALE); publish(); return; }
    if ((ins_status & s_cfg.ins_status_mask) != s_cfg.ins_status_mask) {
        set_fault(GDS_FAULT_INS_INVALID); publish(); return;
    }
    if (!gds_read_wheel_speeds(wheels, &t_wh) || now - t_wh > s_cfg.wheels_timeout_ms) {
        set_fault(GDS_FAULT_WHEELS_STALE); publish(); return;
    }
    if (!gds_read_accel_x(&accel_g, &t_acc)) accel_g = 0.0f;

    fsae_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.now_ms = now;
    in.gyro_z = s_cfg.fvc_z_sign * gyro_degps * DEG2RAD;
    in.accel_x = s_cfg.accel_x_sign * accel_g * G_MPS2;
    for (int i = 0; i < 4; i++) in.wheel_speed[i] = wheels[i] / s_cfg.vehicle.wheel_radius_m;
    in.crossings = s_crossings;
    if (s_cfg.use_ins_velocity) {
        in.vx_ext = vel_x;
        in.vy_ext = s_cfg.fvc_y_sign * vel_y;
    } else {
        in.vx_ext = NAN;
        in.vy_ext = s_vy_msg;
    }

    fsae_ctrl_step(&s_ctrl, &in, &s_out);
    if (s_out.perception_stale) { set_fault(GDS_FAULT_PERCEPTION_STALE); publish(); return; }

    s_cmd.steer_rad = s_out.steer_rad;
    s_cmd.steer_deg = s_out.steer_rad / DEG2RAD;
    s_cmd.total_drive_torque_nm = s_out.total_drive_torque_nm;
    s_cmd.brake_front_nm = s_out.brake_torque_front_nm;
    s_cmd.brake_rear_nm = s_out.brake_torque_rear_nm;
    s_cmd.valid = 1;
    if (s_out.mission_finished && s_out.vx_est < 0.1f) {
        s_state = GDS_FINISHED;
        s_cmd.total_drive_torque_nm = 0.0f;
        s_cmd.valid = 0;
    }
    publish();
}
