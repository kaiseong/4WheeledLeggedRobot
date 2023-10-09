#include "rmd_motor.h"

/* send command */
    byte frame_off[8]                   = {CMD_MOTOR_OFF,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_multi_angle[8]      = {CMD_READ_MULTI_TURNS_ANGLE,       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_torque_control[8]        = {CMD_TORQUE_CL,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_abs_pos_control[8]       = {CMD_ABS_POSITION_CL,              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_status2[8]          = {CMD_READ_MOTOR_STATUS2,           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_system_reset[8]      = {CMD_SYSTEM_RESET,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_zero_offset[8]       = {CMD_WRITE_ROM_CURRENT_POS_AS_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*command list*/
byte frame_control_mode[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_axis_request[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_vel_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_pos_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_tor_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_limit_control[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_void[8]                  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* RMD method */
RMDmotor::RMDmotor(uint8_t set_id,CAN& can) : id(0x140 | set_id),can(can)
{   }

ODrive::ODrive(uint8_t id,CAN& can): node_id(id), can(can)
{}

bool RMDmotor::sendCMD(){
    txmsg.id=id;
    txmsg.type = CANData;
    memcpy(txmsg.data,cmd,LEN);
    return CAN_OK==can.write(txmsg);
}
bool ODrive::sendCMD(){
    if(!request)
    {
        txmsg.type = CANData;
        memcpy(txmsg.data,cmd,LEN);
    }
    else
        txmsg.type=CANRemote;
    return CAN_OK==can.write(txmsg);
}

/* 0.01A /LSB */ 
void RMDmotor::torqueCL(const int16_t& torque)
{
    *((int16_t*)(frame_torque_control+4)) = torque;
    memcpy(cmd,frame_torque_control,LEN);
    return;
}

/*1DPS/LSB, 0.01degree/LSB*/
void RMDmotor::absposCL(const uint16_t& speed_limit,const int32_t & pos)
{
    *((uint16_t*)(frame_abs_pos_control+2))=speed_limit;
    *((int32_t*)(frame_abs_pos_control + 4)) = pos;
    memcpy(cmd,frame_abs_pos_control,LEN);
    return;
}


/* 0.01 Degree / LSB */
void RMDmotor::read_Multi(){
    void*ptr = memcpy(cmd,frame_read_multi_angle,LEN);
    return;
}

void RMDmotor::reset_zero(){
    memcpy(cmd,frame_zero_offset,LEN);
    return;
}

void RMDmotor::restart(){
    memcpy(cmd,frame_system_reset,LEN);
    return;

}

void RMDmotor::set_cmd(byte input_cmd[]){
    memcpy(cmd,input_cmd,LEN);
    return;
}

void RMDmotor::read_state2(){
    memcpy(cmd,frame_read_status2,LEN);
    return;
}

void ODrive::velocityCL(const float& vel, const float& tor)
{
    txmsg.id = (node_id<<5)|INPUT_VEL;
    *((float*)(frame_vel_control))=vel;
    *((float*)(frame_vel_control+4))=tor;
    request=false;
    memcpy(cmd,frame_vel_control,LEN);
    return;
}

void ODrive::posControl(const float& pos, const int16_t& vel, const int16_t& tor)
{
    txmsg.id = (node_id<<5)|INPUT_POS;
    *((float*)(frame_pos_control))=pos;
    *((int16_t*)(frame_pos_control+4))=vel;
    *((int16_t*)(frame_pos_control+6))=tor;
    request=false;
    memcpy(cmd,frame_pos_control,LEN);
    return;
}

void ODrive::request_encoder()
{
    txmsg.id=(node_id<<5)|ENCODER_ESTIMATE;
    request=true;
    return;
}