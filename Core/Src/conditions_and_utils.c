/*
 * conditions_and_utils.c
 *
 *  Created on: Mar 21, 2025
 *      Author: chris
 */

#include "FVC.h"

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
	if(driving_mode == SLOW_MODE)
		return 110; // 100 Apk, 1/5 the speed
	else
		return 550; // 550 Apk
}

int

void check_button_inputs(){
	//if ready to drive button was pressed --> go into predrive
	//if slow mode button was pressed --> change current limit to slow mode
	static last_slow_mode_button = 0;
	predrive_button = swButon5_state.data;

	if((last_slow_mode_button == 0) || (swButon4_state.data)){
		driving_mode = !driving_mode;
	}
	last_slow_mode_button = swButon4_state.data;
}

uint8_t predrive_conditions_met(){
	uint8_t conditions_met = breakPressureFront_psi.data > PREDRIVE_BREAK_THRESH_psi;
	conditions_met &= (predrive_button == PRESSED);
	conditions_met &= (inputInverterVoltage.data > TS_ON_THRESHOLD_VOLTAGE_V);

	return conditions_met;
}

uint8_t is_vechile_faulting(){
	//check if active: BSPD fault, appsBrakeLatched, Sensors power over current
	//out of range faults on FVC and RVC
	uint8_t fault_tripped = vcuPedalPosition1Fault_state.data;
	fault_tripped |= vcuPedalPosition2Fault_state.data;
	fault_tripped |= vcuBrakePressureSensorFault_state.data;
	fault_tripped |= vcuTractiveSystemCurrentSensorFault_state.data;
	fault_tripped |= vcuPedalPositionCorrelationFault_state.data;
	fault_tripped |= vcuPedalPositionBreakingSensorFault_state.data;

	return fault_tripped;
}

void set_inv_disable(float *desired_current, float *max_current, uint8_t *enable){
	*desired_current = 0;
	*max_current = 0;
	*enable = 0;
}

float calculate_desired_current(){
	float desired_current = ((pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm) * get_max_current_limit();
	desired_current = boundary_check(desired_current, APPS_1_MIN_CURRENT_POS_mm, get_max_current_limit());

	return desired_current;
}
