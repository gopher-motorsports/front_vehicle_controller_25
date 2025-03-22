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

void determine_current_parameters(){
	maxcurrentLimit = is_vechile_faulting() ? 0 : get_max_current_limit();
	desiredCurrent_A = calculate_desired_current();
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

	if(vehicle_state != VEHICLE_DRIVING)
		set_inv_disabled(&desired_current, &max_current, &enable);

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

int get_current_limit(boolean driving_mode){
	if(driving_mode == SLOW_MODE)
		return 100; // 100 A
	else
		return 550; // 400 A
}

// End of vcu.c
