/*
 * conditions_and_utils.c
 *
 *  Created on: Mar 21, 2025
 *      Author: chris
 */

#define NORMAL_MODE 0
#define SLOW_MODE 1

uint8_t driving_mode = NORMAL_MODE;

//Heartbeat LED
void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(HBeat_GPIO_Port, HBeat_Pin);
		last_led = HAL_GetTick();
	}

}


//Get Desired Current Limit
int get_current_limit(boolean driving_mode){
	if(driving_mode == SLOW_MODE)
		return 110; // 100 Apk, 1/5 the speed
	else
		return 550; // 550 Apk
}

void check_button_inputs(){
	//if ready to drive button was pressed --> go into predrive
	//if slow mode button was pressed --> change current limit to slow mode
	static last_RTD_button = 0;
	static last_slow_mode_button = 0;

}

void safety_checks(){
	//check if active: BSPD fault, appsBrakeLatched, Sensors power over current
	//out of range faults on FVC and RVC
	//
}

