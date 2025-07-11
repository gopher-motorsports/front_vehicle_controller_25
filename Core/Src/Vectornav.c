#include "main.h"
#include "Vectornav.h"
#include <stdlib.h>
#include <string.h>
#include "gopher_sense.h"
#include "GopherCAN.h"
//recieved from VectorNav
//double vnTow;
//uint16_t vnWeek;
//uint16_t vnStatus;
//float vnYaw;
//float vnPitch;
//float vnRoll;
//double vnLat;
//double vnLong;
//double vnAlt;
float vnVelN;
float vnVelE;
float vnVelD;
//float vnAttUncertainty;
//float vnPosUncertainty;
//float vnVelUncertainty;

//2.(group).(chunk-byte-offset+1 ).(data-byte-offset)


//imu data
//4.7 (12 bytes)
float vnDeltaVelx; //4.7.0
float vnDeltaVely; //4.7.4
float vnDeltaVelz; //4.7.8

//ins data

//7.1.0
uint16_t insStatus; //bit 0/1 = INS state (0/3 not working, 1 aligning, 2 tracking), bit 7 = GNSS comms error, bit 9 = GNSS compass operational
//2.7.4 VelBody (12 bytes)
float vnBodyVelx; //7.4.0
float vnBodyVely; //7.4.4
float vnBodyVelz; //7.4.8

//used for computation

float imuVel;
uint8_t vnIndBuf[14];
char vnVarBuff[14];
short startIndex = 0;
float highPart, lowPart;
//FLOAT_CAN_STRUCT *vnParams[] = {
//		&vnTowOver_ul,
//		&vnTowUnder_ul
//};

void vnGetVarBuff(uint8_t* vnBuffer, short idx, short length) {
	// idx is the index in the UART message of the first character of the variable
	// calcIdx is the index in the buffer of the first character of the variable
	short calcIdx = (idx + startIndex) % VN_BUFF_SIZE;
	// check if variable is continuous in buffer so we can use one memcpy
	if (calcIdx + length < VN_BUFF_SIZE) {
		memcpy(vnVarBuff, &vnBuffer[calcIdx], length);
	} else {
		memcpy(vnVarBuff, &vnBuffer[calcIdx], VN_BUFF_SIZE - calcIdx);
		memcpy(vnVarBuff + (VN_BUFF_SIZE - calcIdx), &vnBuffer[0], length - (VN_BUFF_SIZE - calcIdx));
	}
	vnVarBuff[length] = '\0';
}

void vnSetVars(uint8_t* vnBuffer) {
	// Find the beginning of the UART message if it is not already found
	if (vnBuffer[startIndex] != '$') {
		startIndex = -1;
		for (short i = 0; i < VN_BUFF_SIZE; i++) {
			if (vnBuffer[i] == '$') {
				startIndex = i;
				break;
			}
		}
		if (startIndex == -1) {
			vnVelN = -1;
			vnVelE = -1;
			vnVelD = -1;
			// TODO: figure out how to throw error
			return;
		}
	}

	// Set the values
//	// vnTow
//	vnGetVarBuff(vnBuffer, 7, 13);
//	vnTow = strtod(vnVarBuff, NULL);
//	// vnWeek
//	vnGetVarBuff(vnBuffer, 21, 4);
//	vnWeek = strtoul(vnVarBuff, NULL, 16);
//	// vnStatus
//	vnGetVarBuff(vnBuffer, 26, 4);
//	vnStatus = strtoul(vnVarBuff, NULL, 16);
//
//	// vnYaw
//	vnGetVarBuff(vnBuffer, 31, 8);
//	vnYaw = strtof(vnVarBuff, NULL);
//	// vnPitch
//	vnGetVarBuff(vnBuffer, 40, 8);
//	vnPitch = strtof(vnVarBuff, NULL);
//	// vnRoll
//	vnGetVarBuff(vnBuffer, 49, 8);
//	vnRoll = strtof(vnVarBuff, NULL);
//	// vnLat
//	vnGetVarBuff(vnBuffer, 58, 12);
//	vnLat = strtod(vnVarBuff, NULL);
//	// vnLong
//	vnGetVarBuff(vnBuffer, 71, 13);
//	vnLong = strtod(vnVarBuff, NULL);
//	// vnAlt
//	vnGetVarBuff(vnBuffer, 85, 10);
//	vnAlt = strtod(vnVarBuff, NULL);
	if(vnVelN == -1){
	// vnVelN
	vnGetVarBuff(vnBuffer, 96, 8);
	vnVelN = strtof(vnVarBuff, NULL);
	}
	// vnVelE
	vnGetVarBuff(vnBuffer, 105, 8);
	vnVelE = strtof(vnVarBuff, NULL);
	// vnVelD
	vnGetVarBuff(vnBuffer, 114, 8);
	vnVelD = strtof(vnVarBuff, NULL);
//
//	// vnAttUncertainty
//	vnGetVarBuff(vnBuffer, 123, 4);
//	vnAttUncertainty = strtof(vnVarBuff, NULL);
//	// vnPosUncertainty
//	vnGetVarBuff(vnBuffer, 128, 4);
//	vnPosUncertainty = strtof(vnVarBuff, NULL);
//	// vnVelUncertainty
//	vnGetVarBuff(vnBuffer, 133, 4);
//	vnVelUncertainty = strtof(vnVarBuff, NULL);
	update_and_queue_param_float(&vnVelN_mps, vnVelN);
	update_and_queue_param_float(&vnVelE_mps, vnVelE);
	update_and_queue_param_float(&vnVelD_mps, vnVelD);
}

//sends all VectorNav variables across can
void vnSendVariables(){
//	update_and_queue_param_u16(&vnWeek_ul, vnWeek);
//	update_and_queue_param_u16(&vnStatus_state, vnStatus);
//	update_and_queue_param_float(&vnYaw_deg, vnYaw);
//	update_and_queue_param_float(&vnPitch_deg, vnPitch);
//	update_and_queue_param_float(&vnRoll_deg, vnRoll);
	update_and_queue_param_float(&vnVelN_mps, vnVelN);
	update_and_queue_param_float(&vnVelE_mps, vnVelE);
	update_and_queue_param_float(&vnVelD_mps, vnVelD);
//	update_and_queue_param_float(&vnAttUncertainty_deg, vnAttUncertainty);
//	update_and_queue_param_float(&vnPosUncertainty_m, vnPosUncertainty);
//	update_and_queue_param_float(&vnVelUncertainty_mps, vnVelUncertainty);

//	vn_send_param_u32(vnParams, vnTow);

}

void update_and_queue_vn_variables(){
	vnSetVars(&vnIndBuf);
	//vnSendVariables();
}
//array gcan_vars holds pointers of higher side at index 0 and lower at index 1
//input val to be split across the two
//void vn_send_param_u32(FLOAT_CAN_STRUCT *gcan_vars[], double inputVal){
//	union {
//	    double u32_val;
//	    float u16_val[2];
//	} data;
//
//    data.u32_val = inputVal;
//    highPart = data.u16_val[0];
//	lowPart = data.u16_val[1];
//
//    update_and_queue_param_u16(gcan_vars[0], highPart);
//    update_and_queue_param_u16(gcan_vars[1], lowPart);
//}

float retVel;
float returnVelocity(float velX,float velY,float velZ){
	retVel = sqrt(velX*velX + velY*velY);
	retVel = sqrt(retVel*retVel + velZ*velZ);
	return retVel;
}
