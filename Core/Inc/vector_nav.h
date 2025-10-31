/*
 * vector_nav.h
 *
 *  Created on: Oct 17, 2025
 *      Author: chris
 */

#ifndef INC_VECTOR_NAV_H_
#define INC_VECTOR_NAV_H_

// >>> Replace with your exact family header (e.g., stm32f7xx_hal.h)
#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
	uint16_t INS_status;
	double Latitude;
	double Longitude;
	double Altitude;
	float VelBodyX;
	float VelBodyY;
	float VelBodyZ;
} vn300_data_group75_t;

typedef struct {
	float Yaw;
	float Pitch;
	float Roll;
	float QuatX;
	float QuatY;
	float QuatZ;
	float QuatS;
	float LinBodyAccX;
	float LinBodyAccY;
	float LinBodyAccZ;
} vn300_data_group76_t;


typedef struct {
	uint8_t TimeUtcY;
	uint8_t TimeUtcMonth;
	uint8_t TimeUtcD;
	uint8_t TimeUtcH;
	uint8_t TimeUtcMin;
	uint8_t TimeUtcS;
	uint16_t TimeUtcF;
	float 	GyroBodyX;
	float   GyroBodyY;
	float 	GyroBodyZ;
} vn300_data_group77_t;

extern volatile vn300_data_group75_t vn300_75;
extern volatile vn300_data_group76_t vn300_76;
extern volatile vn300_data_group77_t vn300_77;

// Start non-blocking RX on the given UART (e.g., &huart1)
void vn300_start_rx(UART_HandleTypeDef *huart);
void init_vnav_uart(UART_HandleTypeDef *huart_debug, UART_HandleTypeDef *huart_nav);

#endif /* INC_VECTOR_NAV_H_ */
