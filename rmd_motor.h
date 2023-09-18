#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include "mbed.h"

using namespace mbed;

/* result communication */
#define CAN_FAILED                          0
#define CAN_OK                              1



/* command list */
#define CMD_WRITE_ROM_CURRENT_POS_AS_ZERO   0x64
#define CMD_SYSTEM_RESET                    0x76
#define CMD_READ_MULTI_TURNS_ANGLE          0x92
#define CMD_READ_MOTOR_STATUS2              0x9C
#define CMD_TORQUE_CL                       0xA1
#define CMD_SPEED_CL                        0xA2
#define CMD_ABS_POSITION_CL                 0xA4
#define CMD_MOTOR_OFF                       0x80

/* direction */
#define CMD_MOTOR_DIR_CCW                   0x01
#define CMD_MOTOR_DIR_CW                    0x00

/* etc */
#define LEN                                 8

/* send command */
    extern    byte frame_off[8];                     
    extern    byte frame_read_multi_angle[8];        
    extern    byte frame_torque_control[8];     
    extern    byte frame_abs_pos_control[8];       
    extern    byte frame_read_status2[8];          
    extern    byte frame_system_reset[8];      
    extern    byte frame_zero_offset[8];       

typedef struct motorState2{
    uint8_t cmd;
    int8_t temperature;
    int16_t iq;
    int16_t speed;
    int16_t degree;
} motor_state2;

typedef struct{
    uint32_t dummy;
    int32_t degree;
} multi_angle;


class RMDmotor{
private:
    uint16_t id;
    CAN& can;
    CANMessage rxmsg;
    CANMessage txmsg;
    motor_state2 state;
    multi_angle multi;
    byte cmd[8] = {0x00,};
    bool sendCMD();

public:
    RMDmotor(uint8_t set_id, CAN& can);
    bool torqueCL(const int16_t& torque);
    bool read_Multi();
    bool reset_zero();
    bool restart();
    bool set_cmd(byte input_cmd[]);
    bool read_state2();

/* return motor states(inline) */
    inline const uint16_t getId()                     {return id;}
    inline  byte* receive_state()                {return (byte*)&state;}
    inline  byte* receive_multi()                {return (byte*)&multi;}
    inline  byte* getCMD()                       {return (byte*)&cmd;} 
    inline const int8_t &getTemeperatureRAW()   {return state.temperature;}
    inline const int16_t &getIqRAW()            {return state.iq;}
    inline const int16_t &getSpeedRAW()         {return state.speed;}
    inline const int16_t &getDegreeRAW()        {return state.degree;}
    inline int32_t getMultiAngleRAW()           {return multi.degree;}
};

#endif 