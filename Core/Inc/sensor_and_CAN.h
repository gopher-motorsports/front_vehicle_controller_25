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
void update_inverter_params(uint8_t vehicle_state, float desired_current, float max_current, float max_dc_current, uint8_t enable);
void update_pedal_percent();
void calculate_rpm();
float clamp(float data, float min, float max);
void update_display_fault_status();
void update_sdc_params();

#endif /* INC_SENSOR_AND_CAN_H_ */
