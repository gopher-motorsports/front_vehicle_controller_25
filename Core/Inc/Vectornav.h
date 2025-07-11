#ifndef VECTORNAV_H
#define VECTORNAV_H

// Buffer size of vectornav UART messages
#define VN_BUFF_SIZE 142

void vnSetVars(uint8_t* vnBuffer);
void vnSendVariables();
void update_and_queue_vn_variables();
float returnVelocity(float velX,float velY,float velZ);
//void vn_send_param_u32(FLOAT_CAN_STRUCT *gcan_vars[], double inputVal);
#endif
