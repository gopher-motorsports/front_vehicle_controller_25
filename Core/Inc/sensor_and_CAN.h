/*
 * sensor_and_CAN.h
 *
 *  Created on: Apr 3, 2025
 *      Author: chris
 */

#ifndef INC_SENSOR_AND_CAN_H_
#define INC_SENSOR_AND_CAN_H_
void update_periodic_CAN_params();
void update_fault_state(U8_CAN_STRUCT *param, uint8_t state, uint8_t last_state);
void update_inverter_params(uint8_t vehicle_state, float desired_current, float max_current, uint8_t enable);
void update_pedal_percent();
void calculate_rpm();
float boundary_check(float data, float min, float max);
void update_display_fault_status();


#endif /* INC_SENSOR_AND_CAN_H_ */
