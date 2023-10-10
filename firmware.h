#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "rmd_motor.h"
#include "mbed.h"
using namespace rtos;
using namespace mbed;

/* result communication */
#define CAN_FAILED                          0
#define CAN_OK                              1


/* pin set */
#define CAN_RX                              PB_5
#define CAN_TX                              PB_13


/* etc */
#define LEN                                 8
#define CAN_INTERVAL                        350
#define CAN_CNT                             12

#define BASIC_SPEED                         500



enum RMD_STATE { START, TORQUE, MULTI, POSITION, IDLE};

class Controller{
private:
    RMDmotor rmd[4];
    ODrive odrv[4];
    
    CAN can;
    CANMessage rxmsg;
    CANMessage txmsg;
    byte dummy[8] ={0,};
    uint32_t count=0;
/* receive method */

/* send command */
public:
    
    Controller();
    /*common*/
    void init();
    void receive();
    void run(const int32_t rmd_pos[],const float odrv_vel[], const float Kp[], const float Ki[]);
    void check();
    /*RMD*/
    void rmd_reset();
    /*ODrive*/
};


#endif

