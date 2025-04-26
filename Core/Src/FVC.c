/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include <FVC.h>
#include <stdlib.h>

//GopherLibraries
#include "gopher_sense.h"

//Suporting Files:
#include "conditions_and_utils.h"
#include "sensor_and_CAN.h"
#include "main.h"

//HAL Files
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

CAN_HandleTypeDef* hcan;
uint32_t preDriveTimer_ms = 0;

float desiredCurrent_A = 0;
float maxcurrentLimit_A = 0;

boolean Current_Fault_3V3_state = 0;
boolean Current_Fault_5V_state = 0;

boolean current_driving_mode = 0;
boolean readyToDriveButtonPressed_state = 0;
boolean current_mode_button_state = 0;
boolean past_mode_button_state = 0;

VEHICLE_STATE_t vehicle_state = VEHICLE_NO_COMMS;
U8 inverter_enable_state = INVERTER_DISABLE;
boolean vehicle_currently_moving = 0;
LAUNCH_CONTROL_STATES_t launch_control_state = LAUNCH_CONTROL_DISABLED;


#define HBEAT_LED_DELAY_TIME_ms 500
#define RTD_DEBOUNCE_TIME_ms 25
//#define SET_INV_DISABLED() do{ desiredCurrent_A = 0; maxcurrentLimit_A = MAX_TEST_CMD_CURRENT_A; inverter_enable_state = INVERTER_DISABLE; } while(0)

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN0);
}
float motor_temp = 0;

void main_loop() {
	LED_task();
	maxCurrentLimitPeakToPeak_A.data = 125;
	send_group(0x10E);

	//update_and_queue_param_u8(&driveEnable_state, 1);
	//update_periodic_CAN_params();
	//determine_current_parameters();
	//update_display_fault_status();
	//process_inverter();
	//motor_temp = 0;
}

/**
 * Services the CAN RX and TX hardware task
 */
void can_buffer_handling_loop()
{
	// Handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// An error has occurred
	}

	// Handle the transmission hardware for each CAN bus
	service_can_tx(hcan);
}

void determine_current_parameters(){
	maxcurrentLimit_A = is_vechile_faulting() ? 0 : get_max_current_limit();
	desiredCurrent_A = calculate_desired_current();
}

void process_inverter() {

	/*if(faultCode.data != 0x00) {
		vehicle_state = VEHICLE_FAULT;
	}*/

	if(vehicle_state != VEHICLE_DRIVING)
		set_inv_disabled(&desiredCurrent_A, &maxcurrentLimit_A, &inverter_enable_state);

	switch (vehicle_state)
	{
	case VEHICLE_NO_COMMS:
		// check too see we are receiving messages from inverter to validate comms
		if ((HAL_GetTick() -  driveEnableInvStatus_state.info.last_rx) < INVERTER_TIMEOUT_ms)
		{
			vehicle_state = VEHICLE_STANDBY;
		}

		break;

	case VEHICLE_FAULT:
		//check to see if fault goes away
		if(faultCode.data == 0x00) {
			vehicle_state = VEHICLE_NO_COMMS;
		}

		break;

	case VEHICLE_STANDBY:
		// everything is good to go in this state, we are just waiting to enable the RTD button
		if (predrive_conditions_met()){
			vehicle_state = VEHICLE_PREDRIVE;
			preDriveTimer_ms = 0;
		}

		break;

	case VEHICLE_PREDRIVE:
		// buzz the RTD buzzer for the correct amount of time
		if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}

		break;

	case VEHICLE_DRIVING:
		if(inputInverterVoltage_V.data < TS_ON_THRESHOLD_VOLTAGE_V){
			vehicle_state = VEHICLE_NO_COMMS;
		}
		// vehcile in driving state
#ifdef USING_LAUNCH_CONTROL
		launch_control_sm();
#endif
		inverter_enable_state = INVERTER_ENABLE;
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;

		break;
	}

	// send the current request
	update_inverter_params(vehicle_state, desiredCurrent_A, maxcurrentLimit_A, inverter_enable_state);
}


void launch_control_sm(){
//	float new_current_limit;
//	switch(launch_control_state){
//	case LAUNCH_CONTROL_DISABLED:
//		if(!vehicle_currently_moving)
//			launch_control_state = LAUNCH_CONTROL_ENABLED;
//		break;
//	case LAUNCH_CONTROL_ENABLED:
//		if ((motor_rpm < MIN_LIMIT_SPEED_rpm) && (inputInverterVoltage_V.data != 0))
//		{
//			//rearrange power = toruqe * rpm for current --> I = (torque*rpm) /V
//			new_current_limit = (MAX_LAUNCH_CONTROL_TORQUE_LIMIT * ((motor_rpm * MATH_TAU) / SECONDS_PER_MIN) ) / inputInverterVoltage_V.data;
//		}
//		else{
//			launch_control_state = LAUNCH_CONTROL_DISABLED;
//			break;
//		}
//		if (new_current_limit < maxcurrentLimit_A) maxcurrentLimit_A = new_current_limit;
//
//		break;
//	}
}

// End of vcu.c
