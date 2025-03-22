/*
 * sensor_processing.c
 *
 *  Created on: Feb 13, 2025
 *      Author: chris
 */

#include "FVC.h"
#include "gopher_sense.h"
#include "stdlib.h"


//Always Periodic --> pedalPosition1 %, pedalPosition2 %, wheel speed front left, wheel speed front right, out of range --> apps1, apps2, brake front, cor
//Inverter State Machine Periodic --> desired_current, max_current, enable state, vehicle state
//Change based --> vcuPedalPosition1Fault_state, vcuPedalPosition2Fault_state, vcuBrakePressureSensorFault_state, vcuTractiveSystemCurrentSensorFault_state
// vcuPedalPositionCorrelationFault_state, vcuPedalPositionBrakingFault_state

FLOAT_CAN_STRUCT *periodic_float_params[] = {&pedalPosition1_percent, &pedalPosition2_percent};
uint8_t float_params_len;

void update_periodic_CAN_params(){
	update_pedal_percent();
	for(int i = 0; i < float_params_len; i++){
		update_and_queue_param_float(periodic_float_params[i], periodic_float_params[i]->data);
	}
}

void update_fault_state(U8_CAN_STRUCT *param, uint8_t state, uint8_t last_state){
	if(last_state != state)
		update_and_queue_param_u8(param, state);
}

void update_inverter_params(uint8_t vehicle_state, float desired_current, float max_current, uint8_t enable){
	//update global vehicle state, enable, desired/max current, ready to drive buzzer
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
	if(vehicle_state == VEHICLE_PREDRIVE)
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);

	update_and_queue_param_float(&desiredInvCurrentPeakToPeak_A, desired_current);
	update_and_queue_param_float(maxCurrentLimitPeakToPeak_A, max_current);
	update_and_queue_param_u8(driveEnable_state, enable);
}

void update_display_fault_status(){

}

void update_pedal_percent(){
	float pedal_pos1_percent = 100.0*(pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm;
	pedal_pos1_percent = boundary_check(pedal_pos1_percent, 0.0, 100.0);

	float pedal_pos2_percent = 100.0*(pedalPosition2_mm.data-APPS_2_MIN_CURRENT_POS_mm)/APPS_2_TOTAL_TRAVEL_mm;
	pedal_pos2_percent = boundary_check(pedal_pos2_percent, 0.0, 100.0);
}

float calculate_rpm(){
	motor_rpm = electricalRPM_erpm.data * MOTOR_POLE_PAIRS;
	wheelSpeedRearRight_mph.data = ((motor_rpm * MINUTES_PER_HOUR) * WHEEL_DIAMETER_IN * MATH_PI) / (FINAL_DRIVE_RATIO * IN_PER_FT);
	wheelSpeedFrontLeft_mph.data = wheelSpeedFrontRight_mph.data;
}

float boundary_check(float data, float min, float max){
	if(data < min){
		return min;
	}

	else if (data > max){
		return max;
	}

	return data;
}
