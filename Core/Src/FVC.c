/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include <FVC.h>
#include <sensor_overcurrent_faults.h>
#include "gopher_sense.h"
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

uint32_t maxCurrent_mA = 0;
uint32_t preDriveTimer_ms = 0;

float desiredCurrent_A = 0;
float maxcurrentLimit_A = 0;
float bspd_power_limit = 4000;
//float desiredTorque_Nm = 0;
float torqueLimit_Nm = 0;

float motor_rpm;

boolean appsBrakeLatched_state = 0;

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

//Cooling variables

// Test Temps
float TEST_TEMP_INVERTER = 27.0;
float TEST_TEMP_MOTOR = 27.0;
float test_rpm = 0;

//Fan
boolean steady_temperatures_achieved_fan[] = {true, true}; //LOT if fan temperatures have returned to steady state, implemented to stop double counting
U8 fan_readings_below_HYS_threshold = 0;

//Pump
TIM_HandleTypeDef* PUMP_PWM_Timer;
U32 PUMP_Channel;

boolean steady_temperatures_achieved_pump[] = {true, true}; //LOT if pump temperatures have returned to ready state
U8 pump_readings_below_HYS_threshold = 0;



#define HBEAT_LED_DELAY_TIME_ms 500
#define RTD_DEBOUNCE_TIME_ms 25
//#define SET_INV_DISABLED() do{ desiredCurrent_A = 0; maxcurrentLimit_A = MAX_TEST_CMD_CURRENT_A; inverter_enable_state = INVERTER_DISABLE; } while(0)

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {

	process_sensors();
	process_inverter();
	update_outputs();
	update_display_fault_status();
	update_gcan_states(); // Should be after proceass_sensors
	LED_task();
	vehicle_currently_moving = isVehicleMoving();
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

void update_gcan_states() {
	// Log pedal position percentages
	float pedalPos1 = 100.0*(pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm;
	if(pedalPos1 < 0) {
		pedalPos1 = 0;
	} else if (pedalPos1 > 100) {
		pedalPos1 = 100;
	}
	float pedalPos2 = 100.0*(pedalPosition2_mm.data-APPS_2_MIN_CURRENT_POS_mm)/APPS_2_TOTAL_TRAVEL_mm;
	if(pedalPos2 < 0) {
		pedalPos2 = 0;
	} else if (pedalPos2 > 100) {
		pedalPos2 = 100;
	}
	update_and_queue_param_float(&pedalPosition1_percent, pedalPos1);
	update_and_queue_param_float(&pedalPosition2_percent, pedalPos2);

	// VCU software sensors faults, out of range checks
	update_and_queue_param_u8(&vcuPedalPosition1Fault_state, TIMED_SOFTWARE_FAULTS[0]->state);
	update_and_queue_param_u8(&vcuPedalPosition2Fault_state, TIMED_SOFTWARE_FAULTS[1]->state);
	update_and_queue_param_u8(&vcuBrakePressureSensorFault_state, TIMED_SOFTWARE_FAULTS[2]->state);
	update_and_queue_param_u8(&vcuTractiveSystemCurrentSensorFault_state, TIMED_SOFTWARE_FAULTS[3]->state);
	// VCU software safety checks, correlation and APPS/Brake Plausibility check
	update_and_queue_param_u8(&vcuPedalPositionCorrelationFault_state, TIMED_SOFTWARE_FAULTS[4]->state);
	update_and_queue_param_u8(&vcuPedalPositionBrakingFault_state, appsBrakeLatched_state);

	//current requested amps and max amps for DTI Inverter
	update_and_queue_param_float(&vcuCurrentRequested_A, desiredCurrent_A);
	update_and_queue_param_float(&vcuMaxCurrentLimit_A, maxcurrentLimit_A );

	// Vehicle state
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
	update_and_queue_param_u8(&readyToDriveButton_state, readyToDriveButtonPressed_state);
	update_and_queue_param_u8(&vcuGSenseStatus_state, HAL_GPIO_ReadPin(Gsense_GPIO_Port, Gsense_Pin));

	// Calculate wheel speed from rpm, change rpm to
	motor_rpm = electricalRPM_erpm.data * MOTOR_POLE_PAIRS;
	wheelSpeedRearRight_mph.data = ((motor_rpm * MINUTES_PER_HOUR) * WHEEL_DIAMETER_IN * MATH_PI) / (FINAL_DRIVE_RATIO * IN_PER_FT);
	wheelSpeedFrontLeft_mph.data = wheelSpeedFrontRight_mph.data;
}

void process_sensors() {
	maxcurrentLimit_A = get_current_limit(current_driving_mode);

	//Sensor overcurrent Logic, turn off power to inverter if any of the sensor power lines are overcurrenting
#ifdef USING_SOFTWARE_OVERCURRENT_PROT
	Current_Fault_3V3_state = HAL_GPIO_ReadPin(CURR_FAULT_3V3_GPIO_Port, CURR_FAULT_3V3_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low
	Current_Fault_5V_state  = HAL_GPIO_ReadPin(CURR_FAULT_5V_GPIO_Port, CURR_FAULT_5V_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low

	static U32 overcurrent_event_timer_3V3 = 0;
	static U32 overcurrent_event_timer_5V = 0;

	if(Current_Fault_3V3_state){
		overcurrent_event_timer_3V3++;
		if(overcurrent_event_timer_3V3 >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_3V3 = 0;
	}

	if(Current_Fault_5V_state){
		overcurrent_event_timer_5V++;
		if(overcurrent_event_timer_5V >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_5V = 0;
	}
#endif

	desiredCurrent_A = ((pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm) * get_current_limit(current_driving_mode);

	if(pedalPosition1_mm.data < APPS_1_MIN_CURRENT_POS_mm) {
		desiredCurrent_A = 0;
	}

	if(desiredCurrent_A > get_current_limit(current_driving_mode)) {
		desiredCurrent_A =  get_current_limit(current_driving_mode);
	}
}



void update_display_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if (vehicle_state == VEHICLE_FAULT) status = INVERTER_FAULT;
	else if(bmsNumActiveAlerts_state.data) status = BMS_FAULT;
	else if(vcuPedalPositionBrakingFault_state.data) status = RELEASE_PEDAL;
	else if((bspdTractiveSystemBrakingFault_state.data || vcuBrakingClampingCurrent_state.data) && (!BYPASS_ACTIVE)) status = BRAKING_FAULT;
	else if(vcuPedalPositionCorrelationFault_state.data) status = APPS_FAULT;
	else if((bspdFault_state.data
			|| bspdBrakePressureSensorFault_state.data
			|| bspdTractiveSystemCurrentSensorFault_state.data)
			&& (!BYPASS_ACTIVE)) status = BSPD_FAULT;
	else if((vcuBrakePressureSensorFault_state.data
			|| vcuPedalPosition1Fault_state.data
			|| vcuPedalPosition2Fault_state.data
			|| vcuTractiveSystemCurrentSensorFault_state.data) && (!BYPASS_ACTIVE)
			) status = VCU_FAULT;

	update_and_queue_param_u8(&displayFaultStatus_state, status);
}

void process_inverter() {
	U8 inverter_enable_state = INVERTER_DISABLE;

	/*if(faultCode.data != 0x00) {
		vehicle_state = VEHICLE_FAULT;
	}*/

	switch (vehicle_state)
	{
	case VEHICLE_NO_COMMS:
		// check too see we are receiving messages from inverter to validate comms
		if ((HAL_GetTick() -  driveEnableInvStatus_state.info.last_rx) < INVERTER_TIMEOUT_ms)
		{
			vehicle_state = VEHICLE_STANDBY;
		}
		set_inv_disabled();
		break;

	case VEHICLE_FAULT:
		//check to see if fault goes away
		if(faultCode.data == 0x00) {
			vehicle_state = VEHICLE_NO_COMMS;
		}
		set_inv_disabled();
		break;

	case VEHICLE_STANDBY:
		// everything is good to go in this state, we are just waiting to enable the RTD button
		if (brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi &&
				readyToDriveButtonPressed_state &&
				inputInverterVoltage_V.data > TS_ON_THRESHOLD_VOLTAGE_V)
		{
			// Button is pressed, set state to VEHICLE_PREDRIVE
			vehicle_state = VEHICLE_PREDRIVE;
			preDriveTimer_ms = 0;
		}
		set_inv_disabled();
		break;

	case VEHICLE_PREDRIVE:
		// buzz the RTD buzzer for the correct amount of time
		if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}
		set_inv_disabled();
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
		set_inv_disabled();
		break;
	}

	// send the current request
	desiredInvCurrentPeakToPeak_A.data = desiredCurrent_A;
	maxCurrentLimitPeakToPeak_A.data = maxcurrentLimit_A;
	driveEnable_state.data = inverter_enable_state;

	send_group(INVERTER_SET_CURRENT_AC_CMD_ID);
	send_group(INVERTER_MAX_CURRENT_AC_LIMIT_CMD_ID);
	send_group(INVERTER_DRIVE_ENABLE_CMD_ID);
	service_can_tx(hcan);
}


boolean isVehicleMoving() {
    static U32 vehicle_state_timer = 0;

    if (motor_rpm < RPM_LAUNCH_CONTROL_THRESH) {
    	vehicle_state_timer++;
        if(vehicle_state_timer > STOPPED_TIME_THRESH)
        	vehicle_state_timer = STOPPED_TIME_THRESH + 1;

    } else {
        vehicle_state_timer = 0; // Reset the timer
        return TRUE; // Vehicle is moving
    }

    if(vehicle_state_timer > STOPPED_TIME_THRESH){
        return FALSE;
    }

    return TRUE;
}


void launch_control_sm(){
	float new_current_limit;
	switch(launch_control_state){
	case LAUNCH_CONTROL_DISABLED:
		if(!vehicle_currently_moving)
			launch_control_state = LAUNCH_CONTROL_ENABLED;
		break;
	case LAUNCH_CONTROL_ENABLED:
		if ((motor_rpm < MIN_LIMIT_SPEED_rpm) && (inputInverterVoltage_V.data != 0))
		{
			//rearrange power = toruqe * rpm for current --> I = (torque*rpm) /V
			new_current_limit = (MAX_LAUNCH_CONTROL_TORQUE_LIMIT * ((motor_rpm * MATH_TAU) / SECONDS_PER_MIN) ) / inputInverterVoltage_V.data;
		}
		else{
			launch_control_state = LAUNCH_CONTROL_DISABLED;
			break;
		}
		if (new_current_limit < maxcurrentLimit_A) maxcurrentLimit_A = new_current_limit;

		break;
	}
}

void set_inv_disabled(){
	desiredCurrent_A = 0;
	maxcurrentLimit_A = get_current_limit(current_driving_mode);
	inverter_enable_state = INVERTER_DISABLE;
}

int get_current_limit(boolean driving_mode){
	if(driving_mode == SLOW_MODE)
		return 100; // 100 A
	else
		return 550; // 400 A
}

// End of vcu.c
