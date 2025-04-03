/*
 * conditions_and_utils.h
 *
 *  Created on: Mar 21, 2025
 *      Author: chris
 */

#ifndef INC_CONDITIONS_AND_UTILS_H_
#define INC_CONDITIONS_AND_UTILS_H_
#define HBEAT_LED_DELAY_TIME_ms 500
void LED_task();
int get_max_current_limit();
void check_button_inputs();
uint8_t predrive_conditions_met();
uint8_t is_vechile_faulting();
void set_inv_disabled(float *desired_current, float *max_current, uint8_t *enable);
float calculate_desired_current();
boolean isVehicleMoving();

#endif /* INC_CONDITIONS_AND_UTILS_H_ */
