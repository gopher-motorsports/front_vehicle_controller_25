/*
 * sensor_processing.c
 *
 *  Created on: Feb 13, 2025
 *      Author: chris
 */

//Always Periodic --> pedalPosition1 %, pedalPosition2 %, wheel speed front left, wheel speed front right
//Inverter State Machine Periodic --> desired_current, max_current, enable state, vehicle state
//Change based --> vcuPedalPosition1Fault_state, vcuPedalPosition2Fault_state, vcuBrakePressureSensorFault_state, vcuTractiveSystemCurrentSensorFault_state
// vcuPedalPositionCorrelationFault_state, vcuPedalPositionBrakingFault_state

FLOAT_CAN_STRUCT *periodic_float_params[] = {&pedalPosition1_percent, &pedalPosition2_percent};
U8_CAN_STRUCT *periodic_U8_params;

void update_periodic_CAN_params(){

}

void update_fault_state(U8_CAN_STRUCT *param, uint8_t state, uint8_t last_state){
	if(last_state != state)
		update_and_queue_param_u8(param, state);
}

void update_inverter_params(uint8_t vehicle_state, float desired_current, float max_current, uint8_t enable){
	//update global vehicle state, enable, desired/max current, ready to drive buzzer
	send_group(INVERTER_SET_CURRENT_AC_CMD_ID);
	send_group(INVERTER_MAX_CURRENT_AC_LIMIT_CMD_ID);
	send_group(INVERTER_DRIVE_ENABLE_CMD_ID);
}

void update_display_fault_status(){

}

float calculate_pedal_percent(float pedalPos_mm, float min, float max){

}

float calculate_rpm(){
	motor_rpm = electricalRPM_erpm.data * MOTOR_POLE_PAIRS;
	wheelSpeedRearRight_mph.data = ((motor_rpm * MINUTES_PER_HOUR) * WHEEL_DIAMETER_IN * MATH_PI) / (FINAL_DRIVE_RATIO * IN_PER_FT);
	wheelSpeedFrontLeft_mph.data = wheelSpeedFrontRight_mph.data;
}
