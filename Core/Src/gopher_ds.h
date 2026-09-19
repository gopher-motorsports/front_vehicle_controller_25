/*
 * gopher_ds.h - driverless controller integration for the Gopher Motorsports
 * front vehicle controller (STM32F446, FreeRTOS, GopherCAN, Gopher Sense).
 *
 * Wiring:
 *   - Inputs come from GopherCAN parameters defined in go4-26: VectorNav
 *     gyro and body velocities (vnavGyroBodyZ, vnavVelBodyX/Y, vnavINS_status),
 *     longitudinalAccel_G and the four fvcWheelSpeed*_m_per_s values.
 *   - Cone lists arrive on a UART from the perception computer
 *     (fsae_msg framing), fed byte by byte to gds_uart_rx_byte().
 *   - Commands are returned by gds_command() for the FVC's own torque
 *     distribution, and published with the driverless telemetry as the
 *     ds* parameters that add_driverless_params.py adds to the network.
 *
 * Call pattern in the FVC:
 *   gds_init(&cfg);                        once, after gsense_init()
 *   gds_select_mission(event);             when the mission selector changes
 *   gds_go();                              on remote-stop GO in Driverless Ready
 *   gds_uart_rx_byte(b);                   for each UART byte (from a task, not the ISR,
 *                                          or through a ring buffer)
 *   gds_tick();                            every 20 ms
 *   gds_command()->...                     use while gds_state() == GDS_DRIVING
 *   gds_stop();                            on emergency, remote stop or leaving Driving
 *
 * Axis conventions: the controller wants x forward, y left, z up (yaw rate
 * positive counter-clockwise from above). A VectorNav body frame is usually
 * x forward, y right, z down, so y and z are negated by default. VERIFY on the
 * car: drive a slow left-hand circle and check vy_est and the yaw rate the
 * controller logs (dsYawRate_degps) are positive.
 */
#ifndef GOPHER_DS_H
#define GOPHER_DS_H

#include <stdint.h>
#include "fsae_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GDS_IDLE = 0,        /* no mission, or model not loaded */
    GDS_READY = 1,       /* model loaded for the selected mission, waiting for GO */
    GDS_DRIVING = 2,
    GDS_FINISHED = 3,    /* past the finish and stopped */
    GDS_FAULT = 4        /* stop the car: see gds_fault() */
} gds_state_t;

typedef enum {
    GDS_FAULT_NONE = 0,
    GDS_FAULT_MODEL = 1,            /* model slot empty or corrupt */
    GDS_FAULT_WRONG_EVENT = 2,      /* flashed model is for another event */
    GDS_FAULT_INS_STALE = 3,        /* no fresh VectorNav data */
    GDS_FAULT_INS_INVALID = 4,      /* INS status not in the required state */
    GDS_FAULT_WHEELS_STALE = 5,
    GDS_FAULT_PERCEPTION_STALE = 6
} gds_fault_t;

typedef struct {
    const uint8_t      *model_addr;         /* the model slot, e.g. 0x08060000 */
    uint32_t            model_size;         /* slot size, e.g. 0x20000 */
    fsae_vehicle_cfg_t  vehicle;
    float               fvc_y_sign;        /* -1: VectorNav y right -> y left */
    float               fvc_z_sign;        /* -1: VectorNav z down -> z up */
    float               accel_x_sign;       /* +1 if longitudinalAccel_G is positive forward */
    uint8_t             use_ins_velocity;   /* 1: vx, vy from the INS; 0: wheels + IMU, vy = 0 */
    uint16_t            ins_status_mask;    /* bits of vnavINS_status that must be set (0 = none) */
    uint32_t            ins_timeout_ms;     /* e.g. 50 */
    uint32_t            wheels_timeout_ms;  /* e.g. 50 */
} gds_config_t;

typedef struct {
    float   steer_rad;              /* front wheel angle, positive left */
    float   steer_deg;
    float   total_drive_torque_nm;  /* at the wheels, all driven wheels together */
    float   brake_front_nm;         /* front axle total */
    float   brake_rear_nm;          /* rear axle total */
    uint8_t valid;                  /* 0 unless DRIVING with fresh inputs */
} gds_command_t;

void               gds_default_config(gds_config_t *cfg);
int                gds_init(const gds_config_t *cfg);      /* 0 if the model slot is valid */
int                gds_select_mission(uint8_t event);      /* FSAE_EVENT_*; 0 if the model matches */
void               gds_go(void);
void               gds_stop(void);
void               gds_uart_rx_byte(uint8_t byte);
void               gds_tick(void);
const gds_command_t *gds_command(void);
gds_state_t        gds_state(void);
gds_fault_t        gds_fault(void);
const fsae_policy_header_t *gds_model(void);              /* NULL if none loaded */

/* Read-only views for logging and host tests. */
const fsae_outputs_t *gds_debug_outputs(void);             /* the last controller step */
const fsae_ctrl_t    *gds_debug_ctrl(void);                /* controller state (frame times, pose) */

/* Inputs, overridable (weak) if the FVC reads the VectorNav directly rather
 * than through GopherCAN parameters. Times are HAL ticks (ms). Return 0 if
 * the value is not available. */
int gds_read_ins(float *gyro_z_degps, float *vel_x, float *vel_y, uint16_t *status, uint32_t *t_ms);
int gds_read_accel_x(float *accel_g, uint32_t *t_ms);
int gds_read_wheel_speeds(float mps[4], uint32_t *t_ms);   /* FL, FR, RL, RR */
uint32_t gds_millis(void);

#ifdef __cplusplus
}
#endif
#endif /* GOPHER_DS_H */
