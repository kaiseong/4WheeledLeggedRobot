#ifndef ODRIVE_MOTOR_H
#define ODRIVE_MOTOR_H

#include "mbed.h"
#include "interface.h"
using namespace mbed;

#define LEN                                 8

/* result communication */
#define CAN_FAILED                          0
#define CAN_OK                              1

/* command list */
#define ODRIVE_HEARTBEAT                    0x01
#define MOTOR_ERROR                         0x03
#define ENCODER_ERROR                       0x04
#define SENSORLESS_ERROR                    0x05
#define AXIS_REQUEST                        0x07
#define ENCODER_ESTIMATE                    0x09
#define ENCODER_COUNT                       0x0A    
#define CONTROL_MODE                        0x0B
#define INPUT_POS                           0x0C
#define INPUT_VEL                           0x0D    
#define INPUT_TOR                           0x0E
#define SET_LIMIT                           0x0F
#define START_ANTICOGGING                   0x10
#define REBOOT                              0x16
#define READ_VOLTAGE                        0x17
#define CLEAR_ERROR                         0x18 

/*command list*/
    extern    byte frame_control_mode[8];          
    extern    byte frame_axis_request[8];          
    extern    byte frame_vel_control[8];           
    extern    byte frame_pos_control[8];           
    extern    byte frame_tor_control[8];           
    extern    byte frame_limit_control[8];         
    extern    byte frame_void[8];                  

typedef struct{
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t dummy1;
    uint8_t control_status;
    uint8_t dummy2;
}Heart;

typedef struct{
    float position;
    float velocity;
}Encoder;

class ODrive{
private:
    CAN& can;
    CANMessage rxmsg;
    CANMessage txmsg;
    uint8_t node_id;
    bool receive=false;
    Heart heart;
    Encoder encoder;
    bool sendCMD(uint8_t cmd, byte req[]);
    bool receiveCMD(uint8_t cmd);


public:
    /* Basic */
    ODrive(uint8_t id,CAN& can);

    /* Read function */
    bool read_encoder();

    /* Get motor states */
    inline const float& get_pos()               { return encoder.position;}
    inline const float& get_vel()               { return encoder.velocity;}
    inline const uint8_t& get_node_id()             { return node_id;}

    /*address return*/
    inline byte* receive_heart()              { return (byte*)&heart;}
    inline byte* receive_encoder()            { return (byte*)&encoder;}
    
    /* Command function */
    bool set_axis_state(const uint32_t& state);
    bool set_control_mode(const int32_t& control, const int32_t& input);
    bool posControl(const float& pos, const int16_t& vel, const int16_t& tor);
    bool velControl(const float& vel, const float& tor);
    bool torqueControl(const float& torque);
    bool set_limit(const float& vel_limit, const float& cur_limit);
    bool reboot();
    bool clear_error();
};

#endif