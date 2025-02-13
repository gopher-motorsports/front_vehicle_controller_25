/*
 * vcu_software_faults.h
 *
 *  Created on: Mar 7, 2024
 *      Author: chris
 */

#ifndef INC_SENSOR_OVERCURRENT_FAULTS_H_
#define INC_SENSOR_OVERCURRENT_FAULTS_H_
#include <FVC.h>
#include "gopher_sense.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os.h"

#define NUM_OF_TIMED_FAULTS 5
typedef struct {
	float data;
    float max_threshold;
    float min_threshold;
    uint16_t fault_timer;
    uint16_t input_delay_threshold;
    boolean state;
} SOFTWARE_FAULT;

extern SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS];
void update_struct_fault_data();
#endif /* INC_SENSOR_OVERCURRENT_FAULTS_H_ */
