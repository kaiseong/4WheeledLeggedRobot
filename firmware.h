#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "rmd_motor.h"
#include "ODrive_motor.h"
#include "mbed.h"

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

typedef struct{
    uint8_t rmd_mode;
    int16_t rmd_cmd1;
    int16_t rmd_cmd2;
    int16_t rmd_cmd3;
    int16_t rmd_cmd4;
    uint8_t ODrive_mode;
    float odrv_cmd1;
    float odrv_cmd2;
    float odrv_cmd3;
    float odrv_cmd4;
} COMMAND;

enum RMD_STATE { START, TORQUE, MULTI, IDLE};
enum ODRIVE_STATE { VELOCITY, ENCODER};

class Controller{
    private:
    RMDmotor* rmd1;
    RMDmotor* rmd2;
    RMDmotor* rmd3;
    RMDmotor* rmd4;
    ODrive* odrv1;
    ODrive* odrv2;
    ODrive* odrv3;
    ODrive* odrv4;
    RMD_STATE rmd_state=START;
    ODRIVE_STATE odrv_state=VELOCITY;
    COMMAND command={0,0,0,0,0,0,0.0,0.0,0.0,0.0};
    Timer tmr;
    CAN can;
    CANMessage rxmsg;
    CANMessage txmsg;
    bool run=true;
    bool sent=false;
    uint8_t send_cnt=0;
    uint32_t secnt1,secnt2,secnt3,secnt4;
    uint32_t recnt1,recnt2,recnt3,recnt4;
/* receive method */
    void receive_state();
    void receive_multi();

/* send command */
    byte dummy[8] ={0,};
/* checking array */
    bool okay[4] = {true, true, true, true};
    bool ready[4] = {false, false, false, false};

public:
    bool check[4] = {false, false, false, false};
    Controller();
    ~Controller();
    void init();
    void checking();
    void run();
    /*RMD*/
    void rmd_reset();
    void rmd_torque(int16_t cmd[]);
    void rmd_multi();
    void rmd_state();
    /*ODrive*/
    void odrv_vel();
    void odrv_encoder();
    void receve_encoder();
    void cas();
};


#endif

