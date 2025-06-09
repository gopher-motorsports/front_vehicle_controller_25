/*
 * fvc_software_faults.c
 *
 *  Created on: May 1, 2025
 *      Author: chris
 */

#include "fvc_software_faults.h"
#include "gopher_sense.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include <float.h>
#include <math.h>


// ======================================= Out of Range Checks ======================================
//APPS1 Out of Range Check
SOFTWARE_FAULT APPS1_Range_Fault = {
    .data = 0,
	.max_threshold = APPS_MAX_ERROR_POS_mm,
	.min_threshold = APPS_MIN_ERROR_POS_mm,
	.fault_timer = 0,
	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
	.state = false
};

//APPS2 Out of Range Check
SOFTWARE_FAULT APPS2_Range_Fault = {
    .data = 0,
	.max_threshold = APPS_MAX_ERROR_POS_mm,
	.min_threshold = APPS_MIN_ERROR_POS_mm,
	.fault_timer = 0,
	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
	.state = false
};

//Brake Pressure Sensor Out of Range Check
//SOFTWARE_FAULT BRK_PRESSURE_Range_Fault = {
//    .data = 0,
//	//.max_threshold = BRAKE_PRESS_MAX_psi,
//	//.min_threshold = BRAKE_PRESS_MIN_psi,
//	.max_threshold = BYPASS_MAX,
//	.min_threshold = BYPASS_MIN,
//	.fault_timer = 0,
//	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
//	.state = false
//};

//Tractive System Current Sensor Out of Range Check
//SOFTWARE_FAULT TS_CURRENT_Range_Fault = {
//    .data = 0,
////	.max_threshold = TS_CURRENT_MAX_A,
////	.min_threshold = TS_CURRENT_MIN_A,
//	.max_threshold = BYPASS_MAX,
//	.min_threshold = BYPASS_MIN,
//	.fault_timer = 0,
//	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
//	.state = false
//};

// =================================== APPS1/APPS2 Correlation Check ===============================
SOFTWARE_FAULT Pedal_Correlation_Fault = {
    .data = 0, //float absolute value
	.max_threshold = APPS_CORRELATION_THRESH_mm,
	.min_threshold = -FLT_MIN, //not using this one, put in smallest mimumum value of a float sowon't trigger
	.fault_timer = 0,
	.input_delay_threshold = CORRELATION_TRIP_DELAY_ms,
	.state = false
};
//look at 100 fault
SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS] = {
    &APPS1_Range_Fault,
    &APPS2_Range_Fault,
    //&BRK_PRESSURE_Range_Fault,
    //&TS_CURRENT_Range_Fault,
    &Pedal_Correlation_Fault,
};

void update_struct_fault_data(){
	APPS1_Range_Fault.data = pedalPosition1_mm.data;
	APPS2_Range_Fault.data = pedalPosition2_mm.data;
	//BRK_PRESSURE_Range_Fault.data = brakePressureFront_psi.data;
	//TS_CURRENT_Range_Fault.data = rvcTractiveSystemCurrent_A.data;
	Pedal_Correlation_Fault.data = fabsf(pedalPosition1_percent.data - pedalPosition2_percent.data);
}
