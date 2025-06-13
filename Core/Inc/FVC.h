/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_FVC_H_
#define INC_FVC_H_

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

// ========================================== CONSTANTS =========================================
#define MATH_PI           3.14159265
#define MATH_TAU          MATH_PI*2
#define SECONDS_PER_MIN   60
#define MINUTES_PER_HOUR  60     // For conversion from rpm to mph
#define IN_PER_FT         12     // For conversion from rpm to mph
// ==============================================================================================

// ======================================= APPS PARAMETERS ======================================
#define APPS_MAX_TORQUE_POS_mm  21.5f // The position of the pedal at 100% torque
#define APPS_MIN_TORQUE_POS_mm  2.5f  // The position of the pedal at 0% torque
//#define APPS_MAX_ERROR_POS_mm 24.0f // position where the error begins
//#define APPS_MIN_ERROR_POS_mm 1.0f // position where the error begins
//#define APPS_TOTAL_TRAVEL_mm ( APPS_MAX_TORQUE_POS_mm - APPS_MIN_TORQUE_POS_mm )

#define APPS_1_MAX_CURRENT_POS_mm  21.61f // The position of the pedal at 100% torque
#define APPS_1_MIN_CURRENT_POS_mm  6.18f  // The position of the pedal at 0% torque
#define APPS_2_MAX_CURRENT_POS_mm  19.53f // The position of the pedal at 100% torque
#define APPS_2_MIN_CURRENT_POS_mm  3.94f  // The position of the pedal at 0% torque
#define APPS_MAX_ERROR_POS_mm 23.00f // position where the error begins, check back on this
#define APPS_MIN_ERROR_POS_mm 1.00f  // position where the error begins
#define APPS_1_TOTAL_TRAVEL_mm ( APPS_1_MAX_CURRENT_POS_mm - APPS_1_MIN_CURRENT_POS_mm )
#define APPS_2_TOTAL_TRAVEL_mm ( APPS_2_MAX_CURRENT_POS_mm - APPS_2_MIN_CURRENT_POS_mm )
// ==============================================================================================

// ==============================================================================================
// ============================= TRACTIVE SYSTEM CURRENT PARAMETERS =============================
#define MIN_LIMIT_SPEED_rpm 1000 // minimum RPM that the VCU begins current limiting
// =============================================================================================

// ================================== READY TO DRIVE PARAMETERS =================================
#define PREDRIVE_BRAKE_THRESH_psi  10  // The minimum brake pressure to enter the driving state
#define PREDRIVE_BUTTON_PRESSED    1    // The value of the button parameter when pressed
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define RTD_BUTTON_PUSHED          (GPIO_PIN_RESET)
#define TS_ON_THRESHOLD_VOLTAGE_V  40
// ==============================================================================================


// ====================================== SAFETY PARAMETERS =====================================
// -------------------------------------- Input Validation --------------------------------------
#define INPUT_TRIP_DELAY_ms 85  // The amount of time it takes an input fault to take effect

// -------------------------------------- APPS/Brake Check --------------------------------------
// This check is done using APPS1 (since APPS1 determines the applied torque) and the BSE
#define APPS_BRAKE_PRESS_THRESH_psi  100.0f  // The minimum amount of brake pressure that will trip
// The minimum APPS position that will trip the APPS/Brake check
#define APPS_BRAKE_APPS1_THRESH_mm   ( APPS_1_TOTAL_TRAVEL_mm * 0.25 ) + APPS_1_MIN_CURRENT_POS_mm
// The maximum APPS position that will reset the APPS/Brake check
#define APPS_BRAKE_RESET_APPS1_THRESH_mm   ( APPS_1_TOTAL_TRAVEL_mm * 0.05 ) + APPS_1_MIN_CURRENT_POS_mm

// ------------------------------------ APPS Correlation Check ----------------------------------
//#define APPS_CORRELATION_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.1 )
#define APPS_CORRELATION_THRESH_mm   10
#define CORRELATION_TRIP_DELAY_ms    85  // The amount of time it takes a correlation fault to take effect
#define APPS_CORRELATION_OFFSET_mm   0.0  // The average offset of the APPSs, positive is APPS1 > APPS2

// ------------------------------------ TS Current/Brake Check ----------------------------------
#define BRAKE_TS_CURRENT_THRESH_A  14.0f  // The current limit when the brake and
#define BRAKE_TS_PRESS_THRESH_psi  450.0f // The amount of brake pressure needed
#define BRAKE_TS_MAX_TORQUE            // A clamp on the maximum amount of torque when triggered
#define BRAKE_TS_ON_DELAY_ms       50  // The amount of timer it takes the limit to turn on
#define BRAKE_TS_OFF_DELAY_ms      50  // The amount of timer it takes the limit to turn off
// ==============================================================================================

// =============================== SENSOR OVERCURRENT PARAMETERS ================================
#define SENSOR_OVERCURRENT_TRIPPED      (GPIO_PIN_RESET)
#define SENSOR_OVERCURRENT_TIME_THRESH 	5
//#define USING_SOFTWARE_OVERCURRENT_PROT

// ================================== TRACTIVE SYSTEM PARAMETERS 2023 ================================
#define MOTOR_DIRECTION         1      // Motor direction; 0 is reverse, 1 is forward
#define MAX_CMD_TORQUE_Nm       150.0f    // The maximum torque that will be commanded
#define INVERTER_TIMEOUT_ms     200    // The time after which the vehicle will enter STARTUP
#define INVERTER_ENABLE         1      // Flags to enable the inverter
#define INVERTER_DISABLE        0   	// Flags to disable the inverter
#define INVERTER_LOCKOUT        0x80   // Lockout is bit 7 of byte 6
#define INVERTER_CMD_ID         0x0C0  // The CAN ID of the inverter command
#define INVERTER_PARAM_ID       0x0C1  // The CAN ID of the parameter message
#define INVERTER_NO_FAULT       0x000  // The data of the fault code when there is no inverter fault
#define PARAM_CMD_FAULT_CLEAR   20     // Address of the fault clear parameter
#define PARAM_CMD_READ          0      // Value to send in parameter command to read value
#define PARAM_CMD_WRITE         1      // Value to send in parameter command to read value
#define PARAM_CMD_RESERVED1     0x00   // Reserved value in inverter parameter
#define PARAM_FAULT_CLEAR_DATA  0      // Value to send in the data field when clearing faults
#define PARAM_CMD_RESERVED2     0x0000 // Reserved value in inverter parameter
#define FINAL_DRIVE_RATIO       4.363  // The final drive ratio
#define WHEEL_DIAMETER_IN       10     // Wheel diameter
#define MOTOR_POLE_PAIRS   		10 	   // Amount of Pole Pairs of the EMRAX Motor
#define BSPD_POWER_LIMIT		4000   // 5 kW limit before a car shutdown is required, 4 kW with buffer
#define MAX_DC_CURRENT_LIMIT    200
// ==============================================================================================

// ================================== TRACTIVE SYSTEM PARAMETERS 2024 ================================
#define INVERTER_DRIVE_ENABLE_CMD_ID         0x18E  // The CAN ID for Drive Enable Command
#define INVERTER_MAX_CURRENT_AC_LIMIT_CMD_ID 0x10E  // The CAN ID for Setting Max Current Limit
#define INVERTER_SET_CURRENT_AC_CMD_ID     	 0x02E  // The CAN ID for Setting Desired Inverter Current
#define MAX_TEST_CMD_CURRENT_A    			 550  // The maximum current that will be commanded
#define DRIVE_ENABLE_INVERTER_TIMEOUT		 200 //Inverter Timeout if
#define VEHICLE_STOPPED_THRESHOLD			 1000 //If vehicle is stopped for 1 sec
#define SLOW_MODE							 1 // If vehicle is in slow mode;


//#define USING_LAUNCH_CONTROL
#define RPM_LAUNCH_CONTROL_THRESH			10
#define STOPPED_TIME_THRESH					250
#define MAX_LAUNCH_CONTROL_TORQUE_LIMIT	    50	   // 100 nm as max torque when in driving state

#define NORMAL_MODE 0
#define SLOW_MODE 1
// ======================================== I/O PARAMETERS ======================================
#define MOSFET_PULL_DOWN_ON (GPIO_PIN_SET)
#define MOSFET_PULL_DOWN_OFF (GPIO_PIN_RESET)
#define PRESSED 1
#define RELEASED 0
// ==============================================================================================

// ==============================================================================================
#define BYPASS_ACTIVE		TRUE
#define BYPASS_MAX FLT_MAX
#define BYPASS_MIN -FLT_MIN
// ======================================= BYPASS PARAMETERS ======================================
// Allows us to disable a particular fault
#define BSPD_APPS1_FAULT     GPIO_PIN_RESET
#define BSPD_APPS2_FAULT     GPIO_PIN_RESET
#define BSPD_BRAKE_FAULT     GPIO_PIN_RESET
#define BSPD_TS_SNS_FAULT    GPIO_PIN_RESET
#define BSPD_TS_BRK_FAULT    GPIO_PIN_SET
// ==============================================================================================


// =================== THROTTLE CALCULATION ===================
// Throttle is calculated using APPS1 with APPS2 being used
// for redundancy in the correlation  check mandated by rules
// ============================================================

// ======================================= BSPD PARAMETERS ======================================
// This is the vehicle state which is affected both by the actions
// of the driver and the current state of the inverter
typedef enum
{
	VEHICLE_NO_COMMS  = 0, // When the inverter first turns on and if there is ever a loss of communication
	VEHICLE_FAULT     = 1, // The vehicle can detect that the inverter is Faulting
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is active and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
} VEHICLE_STATE_t;

typedef enum
{
	LAUNCH_CONTROL_DISABLED,
	LAUNCH_CONTROL_ENABLED
} LAUNCH_CONTROL_STATES_t;
typedef enum {
	NONE = 0,
	RELEASE_PEDAL,
	BRAKING_FAULT,
	APPS_FAULT,
	BSPD_FAULT,
	AMS_FAULT,
	IMD_FAULT,
	VCU_FAULT,
	BMS_FAULT,
	INVERTER_FAULT
} DISPLAY_FAULT_STATUS_t;

extern VEHICLE_STATE_t vehicle_state;


void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();
void process_inverter();   // Updates vehicle state and applicable commands
void launch_control_sm();
void determine_current_parameters();
#endif /* INC_FVC_H_ */
