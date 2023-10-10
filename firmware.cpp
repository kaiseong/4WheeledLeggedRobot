
#include "firmware.h"


/* library method */
Controller::Controller() : can(CAN_RX, CAN_TX), rmd{RMDmotor(2, can), RMDmotor(3, can), RMDmotor(4, can), RMDmotor(5,can)},odrv{ODrive(48,can),ODrive(49,can),ODrive(50,can),ODrive(51,can)}
{
}

void Controller::init()
{
    while (!can.frequency(1000000));
    while (!can.mode(CAN::Normal));
    
}

void Controller::rmd_reset()
{
    for(int i=0;i<4;i++)
    {
        rmd[i].reset_zero();
        rmd[i].sendCMD();
        ThisThread::sleep_for(10);
    }
        for(int i=0;i<4;i++)
    {
        rmd[i].restart();
        rmd[i].sendCMD();
        ThisThread::sleep_for(10);
    }
}


void Controller::receive()
{
    for(int i=0;i<8;i++)
    {
        if(can.read(rxmsg))
        {
            switch(rxmsg.id)
            {
                case 0x242:
                    memcpy(rmd[0].receive_state(),rxmsg.data,LEN);
                    break;
                case 0x243:
                    memcpy(rmd[1].receive_state(),rxmsg.data,LEN);
                    break;
                case 0x244:
                    memcpy(rmd[2].receive_state(),rxmsg.data,LEN);
                    break;
                case 0x245:
                    memcpy(rmd[3].receive_state(),rxmsg.data,LEN);
                    break;
                case 0x609:
                    memcpy(odrv[0].receive_encoder(),rxmsg.data,LEN);
                    break;
                case 0x629:
                    memcpy(odrv[1].receive_encoder(),rxmsg.data,LEN);
                    break;
                case 0x649:
                    memcpy(odrv[2].receive_encoder(),rxmsg.data,LEN);
                    break;
                case 0x669:
                    memcpy(odrv[3].receive_encoder(),rxmsg.data,LEN);
                    break;
            }
        }
    }
}


void Controller::run(const int32_t rmd_pos[],const float odrv_vel[], const float Kp[], const float Ki[]){
    uint8_t i =count%4; 
    if(count%284 <280){
        rmd[i].PID(rmd_pos[i],Kp[i],Ki[i]);
        //rmd[i].absposCL(BASIC_SPEED, rmd_pos[i]);
        rmd[i].sendCMD();
    } 
    else{
        odrv[i].velocityCL(odrv_vel[i]);
        odrv[i].sendCMD();
        check();
    }
    count++;
}

void Controller::check(){
    txmsg.id=0x10;
    txmsg.type = CANData;
    *((int16_t*)(dummy))=rmd[0].getDegreeRAW(); 
    *((int16_t*)(dummy+2))=rmd[1].getDegreeRAW();
    *((int16_t*)(dummy+4))=rmd[2].getDegreeRAW();
    *((int16_t*)(dummy+6))=rmd[3].getDegreeRAW();
    memcpy(txmsg.data,dummy,LEN);
    can.write(txmsg);
}