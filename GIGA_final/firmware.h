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
#define CAN_INTERVAL                        700
#define CAN_CNT                             12

#define BASIC_SPEED                         500

typedef struct{
    uint8_t rmd_mode;
    int16_t rmd_cmd1;
    int16_t rmd_cmd2;
    int16_t rmd_cmd3;
    int16_t rmd_cmd4;
    
} COMMAND;

enum RMD_STATE { START, TORQUE, MULTI, POSITION, IDLE};

class Controller{
    private:
    RMDmotor rmd[4];
    ODrive odrv[4];
    
    Timer rmd_tmr;
    Timer odrv_tmr;
    CAN can;
    CANMessage rxmsg;
    CANMessage txmsg;
    bool rmd_running=false;
    bool rmd_sent=false;
    bool odrv_running=false;
    bool odrv_sent=false;
    bool odrv_readEncoder=false;
    uint8_t send_cnt=0;
    uint32_t secnt1,secnt2,secnt3,secnt4;
    uint32_t recnt1,recnt2,recnt3,recnt4;
    byte dummy[8] ={0,};
/* receive method */

/* send command */
public:
    COMMAND command={START,0,0,0,0};
    Controller();
    void init();
    void rmd_pos(int32_t req[]);
    void odrv_vel(float vel[]);
    void odrv_pos_fix();
    void receive();
    /*RMD*/
    void rmd_send();
    void rmd_reset();
    /*ODrive*/
    void odrv_send();
    void odrv_encoder();
};


#endif

