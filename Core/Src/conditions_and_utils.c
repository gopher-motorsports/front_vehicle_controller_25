/*
 * conditions_and_utils.c
 *
 *  Created on: Mar 21, 2025
 *      Author: chris
 */

#include "FVC.h"
#include "sensor_and_CAN.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

uint8_t driving_mode = NORMAL_MODE;
uint8_t predrive_button = RELEASED;
//Heartbeat LED
void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(HBeat_GPIO_Port, HBeat_Pin);
		last_led = HAL_GetTick();
	}
}

//Get Desired Current Limit
int get_max_current_limit(){
	if(driveSpeedMode_state.data == SLOW_MODE)
		return 55; // 100 Apk, 1/5 the speed
	else
		return 550; // 550 Apk
}

uint8_t predrive_conditions_met(){
	return (brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi) && (PREDRIVE_BUTTON_PARAM.data == PRESSED)
			&& (inputInverterVoltage_V.data > TS_ON_THRESHOLD_VOLTAGE_V);
}

uint8_t is_vechile_faulting(){

	//pulled high when a fault is tripped, intialized to 0
	uint8_t fault_tripped = 0;

	SOFTWARE_FAULT* fault;
	for(int i = 0; i < NUM_OF_TIMED_FAULTS; i++){
		fault = TIMED_SOFTWARE_FAULTS[i];
		if(fault->data > fault->max_threshold || fault->data < fault->min_threshold){ //correlation has no min, but edge case accounted for in defines
			fault->fault_timer++;
			if(fault->fault_timer > fault->input_delay_threshold){
				fault->fault_timer = fault->input_delay_threshold + 1; //cap at delay_threshold + 1 so that it trips but doesn't count up more
			}
		}
		else{
			fault->fault_timer = 0;
			fault->state = false;
		}

		if(fault->fault_timer > fault->input_delay_threshold){
			fault->state = true;
		}

		fault_tripped |= fault->state;
	}

	//input fault = rear brake pressure or current sensor out of range
	fault_tripped |= bspdInputFault_state.data;

	// APPS/Brake Plausibility Fault (both pedals pushed)
	boolean appsBrakeLatched_state;
	if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_percent.data > 25) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_percent.data <= 5) {
		appsBrakeLatched_state = FALSE;
	}

	fault_tripped |= appsBrakeLatched_state;

	return fault_tripped;
}

float calculate_dc_current_limit(){
	float dc_current_limit_A = 0;
	// BSPD Tractive Brake Fault tripped(if this lasts for .5s car the BSPD fault is tripped and HV is shut off)
	if(bspdTractiveSystemBrakingFault_state.data){
		if(inputInverterVoltage_V.data != 0)
			dc_current_limit_A = BSPD_POWER_LIMIT / inputInverterVoltage_V.data; //stay below 5 kW I = P/V
	}
	else{
		dc_current_limit_A = MAX_DC_CURRENT_LIMIT;
	}
	return dc_current_limit_A;
}

void set_inv_disabled(float *max_current, uint8_t *enable){
	if (max_current != NULL) {
		*max_current = 0;
	}
	if (enable != NULL) {
		*enable = 0;
	}
}

float calculate_desired_current(){
	float desired_current = ((pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm) * get_max_current_limit();
	desired_current = clamp(desired_current, 0, get_max_current_limit());
	return desired_current;
}

boolean isVehicleMoving() {
    static U32 vehicle_state_timer = 0;

    //TODO change to non-zero value
    float motor_rpm = 0;
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
