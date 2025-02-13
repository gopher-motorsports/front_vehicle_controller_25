/*
 * drive_control.c
 *
 *  Created on: May 21, 2024
 *      Author: chris
 */
/*
#include "gopher_sense.h"
#include <stdlib.h>
#include "drive_control.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"



VEHICLE_STATE_t vehicle_state = VEHICLE_NO_COMMS;
VEHICLE_STATE_t next_vehicle_state = VEHICLE_NO_COMMS;
float desiredCurrent_A = 0;
void inverter_test_sm();

void inverter_task(){
	vehicle_state = next_vehicle_state;
	inverter_test_sm();
}

void inverter_test_sm(){
	switch (vehicle_state){
			case VEHICLE_NO_COMMS:
				driveEnable_state.data = 1;
				send_group(INVERTER_DRIVE_ENABLE_CMD_ID);
				if(driveEnableInvStatus_state.data == 1 && driveEnableInvStatus_state.info.last_rx < 500){
					next_vehicle_state = VEHICLE_DRIVING;
				}

			case VEHICLE_DRIVING:

				if(faultCode.data != 0x00){
					next_vehicle_state = VEHICLE_FAULT;
				}

				desiredCurrent_A = ((pedalPosition1_mm.data-APPS_MIN_CURRENT_POS_mm)/APPS_TOTAL_TRAVEL_mm) * MAX_TEST_CURRENT_A ;

				if(pedalPosition1_mm.data < APPS_MIN_CURRENT_POS_mm) {
					desiredCurrent_A = 0;
				}

				if(desiredCurrent_A > MAX_TEST_CURRENT_A ) {
					desiredCurrent_A = MAX_TEST_CURRENT_A;
				}

				driveEnable_state.data = 1;
				desiredInvCurrentPeakToPeak_A.data = desiredCurrent_A;

				send_group(INVERTER_DRIVE_ENABLE_CMD_ID);
				send_group(INVERTER_SET_CURRENT_AC_CMD_ID);

			case VEHICLE_FAULT:
				if(faultCode.data == 0x00){
					next_vehicle_state = VEHICLE_DRIVING;
				}

		}
}

void inverter_sm(){
	switch (vehicle_state)
		{
		case VEHICLE_NO_COMMS:
			// wait for comms to try and exit lockout
			if ((HAL_GetTick() - inverterState_state.info.last_rx) < INVERTER_TIMEOUT_ms)
			{
				vehicle_state = VEHICLE_LOCKOUT;
			}
			SET_INV_DISABLED();
			break;

		case VEHICLE_LOCKOUT:
			// try and exit the lockout mode of the inverter. This will be present whenever there is a fault
			if((invStatesByte6_state.data & INVERTER_LOCKOUT)) {
				// We're in lockout; reset faults
				invParameterAddress_state.data = PARAM_CMD_FAULT_CLEAR;
				invParameterRW_state.data = PARAM_CMD_WRITE;
				invParameterReserved1_state.data = PARAM_CMD_RESERVED1;
				invParameterData_state.data = PARAM_FAULT_CLEAR_DATA;
				invParameterReserved2_state.data = PARAM_CMD_RESERVED2;
				send_group(INVERTER_PARAM_ID);
				service_can_tx(hcan);
				vehicle_state = VEHICLE_LOCKOUT;
			} else {
				// We've exited lockout
				vehicle_state = VEHICLE_STANDBY;
			}
			SET_INV_DISABLED();
			break;

		case VEHICLE_STANDBY:
			// everything is good to go in this state, we are just waiting to enable the RTD button
			if (brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi &&
					readyToDriveButtonPressed_state &&
					dcBusVoltage_V.data > TS_ON_THRESHOLD_VOLTAGE_V)
			{
				// Button is pressed, set state to VEHICLE_PREDRIVE
				vehicle_state = VEHICLE_PREDRIVE;
				preDriveTimer_ms = 0;
			}
			SET_INV_DISABLED();
			break;

		case VEHICLE_PREDRIVE:
			// buzz the RTD buzzer for the correct amount of time
			if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
				vehicle_state = VEHICLE_DRIVING;
			}
			SET_INV_DISABLED();
			break;

		case VEHICLE_DRIVING:
			// let the car run as normal. Do not change desiredTorque
			limit_motor_torque();
			inverter_enable_state = INVERTER_ENABLE;
			break;

		default:
			vehicle_state = VEHICLE_NO_COMMS;
			SET_INV_DISABLED();
			break;
		}
}
*/

