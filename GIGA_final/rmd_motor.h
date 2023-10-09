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

/*command list*/
    extern    byte frame_control_mode[8];          
    extern    byte frame_axis_request[8];          
    extern    byte frame_vel_control[8];           
    extern    byte frame_pos_control[8];           
    extern    byte frame_tor_control[8];           
    extern    byte frame_limit_control[8];         
    extern    byte frame_void[8]; 

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

public:
    RMDmotor(uint8_t set_id, CAN& can);
    void torqueCL(const int16_t& torque);
    void absposCL(const uint16_t& speed_limit,const int32_t & pos);
    void read_Multi();
    void reset_zero();
    void restart();
    void set_cmd(byte input_cmd[]);
    void read_state2();
    bool sendCMD();
//    int16_t PID(int16_t ref_pos);

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
    bool request=false;
    Encoder encoder;
    byte cmd[8] = {0x00,};
    

public:
    /* Basic */
    ODrive(uint8_t id,CAN& can);
    bool sendCMD();
    void velocityCL(const float& vel, const float& tor=0);
    void posControl(const float& pos, const int16_t& vel=0, const int16_t& tor=0);
    void request_encoder();
    inline const float& get_pos()               { return encoder.position;}
    inline const float& get_vel()               { return encoder.velocity;}
    inline byte* receive_encoder()              { return (byte*)&encoder;}
    
};


#endif 