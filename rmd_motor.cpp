#include "rmd_motor.h"

/* send command */
    byte frame_off[8]                   = {CMD_MOTOR_OFF,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_multi_angle[8]      = {CMD_READ_MULTI_TURNS_ANGLE,       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_torque_control[8]        = {CMD_TORQUE_CL,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_abs_pos_control[8]       = {CMD_ABS_POSITION_CL,              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_status2[8]          = {CMD_READ_MOTOR_STATUS2,           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_system_reset[8]      = {CMD_SYSTEM_RESET,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_zero_offset[8]       = {CMD_WRITE_ROM_CURRENT_POS_AS_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* RMD method */
RMDmotor::RMDmotor(uint8_t set_id,CAN& can) : id(0x140 | set_id),can(can)
{   }

bool RMDmotor::sendCMD(){
    txmsg.id=id;
    txmsg.type = CANData;
    memcpy(txmsg.data,cmd,LEN);
    return CAN_OK==can.write(txmsg);
}

/* 0.01A /LSB */ 
bool RMDmotor::torqueCL(const int16_t& torque)
{
    *((int16_t*)(frame_torque_control+4)) = torque;
    void*ptr = memcpy(cmd,frame_torque_control,LEN);
    if(ptr==cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}
/* 0.01 Degree / LSB */
bool RMDmotor::read_Multi(){
    void*ptr = memcpy(cmd,frame_read_multi_angle,LEN);
    if(ptr==cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}

bool RMDmotor::reset_zero(){
    void *ptr = memcpy(cmd,frame_zero_offset,LEN);
    if(ptr == cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}

bool RMDmotor::restart(){
    void *ptr=memcpy(cmd,frame_system_reset,LEN);
    if(ptr==cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}

bool RMDmotor::set_cmd(byte input_cmd[]){
    void *ptr = memcpy(cmd,input_cmd,LEN);
    if(ptr==cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}

bool RMDmotor::read_state2(){
    void *ptr = memcpy(cmd,frame_read_status2,LEN);
    if(ptr==cmd){
        return sendCMD();
    }
    return CAN_FAILED;
}