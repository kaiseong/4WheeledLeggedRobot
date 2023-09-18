#include "ODrive_motor.h"

using namespace mbed;

/*command list*/
byte frame_control_mode[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_axis_request[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_vel_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_pos_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_tor_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_limit_control[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte frame_void[8]                  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/* Basic functions */
ODrive::ODrive(uint8_t id,CAN& can): node_id(id), can(can), receive(false)
{}

bool ODrive::receiveCMD(uint8_t cmd){
    txmsg.id=(node_id<<5)|cmd;
    txmsg.type=CANRemote;
    //memcpy(txmsg.data, req, sizeof(req));
    return CAN_OK == can.write(txmsg);
}

bool ODrive::sendCMD(uint8_t cmd, byte req[]){
    txmsg.id = (node_id<<5)|cmd;
    txmsg.type = CANData;
    memcpy(txmsg.data, req, LEN);
    return CAN_OK == can.write(txmsg);
}


/* Update functions */
bool ODrive::read_encoder()
{
    return receiveCMD(ENCODER_ESTIMATE);
}



/* Command functions */
bool ODrive::set_axis_state(const uint32_t& state){
    *((uint32_t*)(frame_axis_request)) = state;
    return sendCMD(AXIS_REQUEST, frame_axis_request);
}

bool ODrive::set_control_mode(const int32_t& control, const int32_t& input){
    *((int32_t*)(frame_control_mode))= control;
    *((int32_t*)(frame_control_mode+4))= input;
    return sendCMD(CONTROL_MODE, frame_control_mode);
}

// vel and tor unit is 0.001
bool ODrive::posControl(const float& pos, const int16_t& vel, const int16_t& tor){
    *((float*)(frame_pos_control))=pos;
    *((int16_t*)(frame_pos_control+4))=vel;
    *((int16_t*)(frame_pos_control+6))=tor;
    return sendCMD(INPUT_POS, frame_pos_control);
}

bool ODrive::velControl(const float& vel, const float& tor){
    *((float*)(frame_vel_control))=vel;
    *((float*)(frame_vel_control+4))=tor;
    return sendCMD(INPUT_VEL, frame_vel_control);
}

bool ODrive::torqueControl(const float& torque){
    *((float*)(frame_tor_control))=torque;
    return sendCMD(INPUT_TOR, frame_tor_control);
}

bool ODrive::set_limit(const float& vel_limit, const float& cur_limit){
    *((float*)(frame_limit_control))=vel_limit;
    *((float*)(frame_limit_control+4))=cur_limit;
    return sendCMD(SET_LIMIT, frame_limit_control); 
}

bool ODrive::reboot(){
    return sendCMD(REBOOT, frame_void);
}

bool ODrive::clear_error(){
    return sendCMD(CLEAR_ERROR, frame_void);
}
