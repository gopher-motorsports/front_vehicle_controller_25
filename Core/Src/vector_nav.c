#include "vector_nav.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

static UART_HandleTypeDef *s_huart = NULL;

#ifndef VN300_RX_DMA_BUF
#define VN300_RX_DMA_BUF 512
#endif

static uint8_t s_dma_buf[VN300_RX_DMA_BUF];
volatile vn300_data_group75_t vn300_75 = {0};
volatile vn300_data_group76_t vn300_76 = {0};
volatile vn300_data_group77_t vn300_77 = {0};

uint16_t offset = 0;
static void process_bytes(const uint8_t *data, uint16_t len) {
	offset = 0;
	for(int i = 0; i < 3; i++){
		//loop until next 0xFA block
		while(data[offset] != 0xFA){
			offset++;
		}

		offset++;
		if(data[offset] == 0x20){
			offset = offset + 3;
			vn300_data_group75_t tmp75;
			memcpy(&tmp75.INS_status, data + offset, sizeof(tmp75.INS_status));
			offset += sizeof(tmp75.INS_status);

			memcpy(&tmp75.Latitude, data + offset, sizeof(tmp75.Latitude));
			offset += sizeof(tmp75.Latitude);

			memcpy(&tmp75.Longitude, data + offset, sizeof(tmp75.Longitude));
			offset += sizeof(tmp75.Longitude);

			memcpy(&tmp75.Altitude, data + offset, sizeof(tmp75.Altitude));
			offset += sizeof(tmp75.Altitude);

			memcpy(&tmp75.VelBodyX, data + offset, sizeof(tmp75.VelBodyX));
			offset += sizeof(tmp75.VelBodyX);

			memcpy(&tmp75.VelBodyY, data + offset, sizeof(tmp75.VelBodyY));
			offset += sizeof(tmp75.VelBodyY);

			memcpy(&tmp75.VelBodyZ, data + offset, sizeof(tmp75.VelBodyZ));
			offset += sizeof(tmp75.VelBodyZ);

			__disable_irq();
			vn300_75 = tmp75;
			__enable_irq();

		}

		else if(data[offset] == 0x10){
			vn300_data_group76_t tmp76;
			offset = offset + 3;

			memcpy(&tmp76.Yaw,          data + offset, sizeof(tmp76.Yaw));
			offset += sizeof(tmp76.Yaw);

			memcpy(&tmp76.Pitch,        data + offset, sizeof(tmp76.Pitch));
			offset += sizeof(tmp76.Pitch);

			memcpy(&tmp76.Roll,         data + offset, sizeof(tmp76.Roll));
			offset += sizeof(tmp76.Roll);

			memcpy(&tmp76.QuatX,        data + offset, sizeof(tmp76.QuatX));
			offset += sizeof(tmp76.QuatX);

			memcpy(&tmp76.QuatY,        data + offset, sizeof(tmp76.QuatY));
			offset += sizeof(tmp76.QuatY);

			memcpy(&tmp76.QuatZ,        data + offset, sizeof(tmp76.QuatZ));
			offset += sizeof(tmp76.QuatZ);

			memcpy(&tmp76.QuatS,        data + offset, sizeof(tmp76.QuatS));
			offset += sizeof(tmp76.QuatS);

			memcpy(&tmp76.LinBodyAccX,  data + offset, sizeof(tmp76.LinBodyAccX));
			offset += sizeof(tmp76.LinBodyAccX);

			memcpy(&tmp76.LinBodyAccY,  data + offset, sizeof(tmp76.LinBodyAccY));
			offset += sizeof(tmp76.LinBodyAccY);

			memcpy(&tmp76.LinBodyAccZ,  data + offset, sizeof(tmp76.LinBodyAccZ));
			offset += sizeof(tmp76.LinBodyAccZ);

			__disable_irq();
			vn300_76 = tmp76;
			__enable_irq();
		 }

		else if(data[offset] == 0x06){
			vn300_data_group77_t tmp77;
			offset = offset + 5;

			memcpy(&tmp77.TimeUtcY,   data + offset, sizeof(tmp77.TimeUtcY));
			offset += sizeof(tmp77.TimeUtcY);

			memcpy(&tmp77.TimeUtcMonth, data + offset, sizeof(tmp77.TimeUtcMonth));
			offset += sizeof(tmp77.TimeUtcMonth);

			memcpy(&tmp77.TimeUtcD,   data + offset, sizeof(tmp77.TimeUtcD));
			offset += sizeof(tmp77.TimeUtcD);

			memcpy(&tmp77.TimeUtcH,   data + offset, sizeof(tmp77.TimeUtcH));
			offset += sizeof(tmp77.TimeUtcH);

			memcpy(&tmp77.TimeUtcMin, data + offset, sizeof(tmp77.TimeUtcMin));
			offset += sizeof(tmp77.TimeUtcMin);

			memcpy(&tmp77.TimeUtcS,   data + offset, sizeof(tmp77.TimeUtcS));
			offset += sizeof(tmp77.TimeUtcS);

			memcpy(&tmp77.TimeUtcF,   data + offset, sizeof(tmp77.TimeUtcF));
			offset += sizeof(tmp77.TimeUtcF);

			memcpy(&tmp77.GyroBodyX,  data + offset, sizeof(tmp77.GyroBodyX));
			offset += sizeof(tmp77.GyroBodyX);

			memcpy(&tmp77.GyroBodyY,  data + offset, sizeof(tmp77.GyroBodyY));
			offset += sizeof(tmp77.GyroBodyY);

			memcpy(&tmp77.GyroBodyZ,  data + offset, sizeof(tmp77.GyroBodyZ));
			offset += sizeof(tmp77.GyroBodyZ);

			__disable_irq();
			vn300_77 = tmp77;
			__enable_irq();
		}
		else{
			continue;
		}
	}
}

// --- public API ---
void vn300_start_rx(UART_HandleTypeDef *huart) {
	__enable_irq();
    s_huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, sizeof(s_dma_buf));
    // Optional: we rely on RxEvent callback; turn off DMA half/TC IT if you like
//    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
//    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_TC);
}

// HAL weak callback; place this in a C file that links (this file is fine)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == s_huart) {
        process_bytes(s_dma_buf, Size);
        memset(&s_dma_buf, 0, sizeof(s_dma_buf));
        // restart reception
        HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, sizeof(s_dma_buf));
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_TC);
    }
}
