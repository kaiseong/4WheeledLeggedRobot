
#include "firmware.h"


/* library method */
Controller::Controller() : can(CAN_RX, CAN_TX), rmd{RMDmotor(2, can), RMDmotor(3, can), RMDmotor(4, can), RMDmotor(5,can)},odrv{ODrive(48,can),ODrive(49,can),ODrive(50,can),ODrive(51,can)}
{
    rmd_tmr.start();
    odrv_tmr.start();
}

void Controller::init()
{
    while (!can.frequency(1000000));
    while (!can.mode(CAN::Normal));
    secnt1=0;
    secnt2=0;
    secnt3=0;
    secnt4=0;
    recnt1=0;
    recnt2=0;
    recnt3=0;
    recnt4=0;
    Serial.begin(115200);
    
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

void Controller::odrv_encoder()
{
    for(int i=0;i<4;i++)
    {
        odrv[i].request_encoder();
        odrv[i].sendCMD();
        ThisThread::sleep_for(1);
    }
    receive();

}


void Controller::rmd_pos(int32_t pos[])
{
    for(int i=0;i<4;i++)
    {
        rmd[i].absposCL(BASIC_SPEED,pos[i]);
    }
}

void Controller::odrv_vel(float vel[])
{
    for(int i=0;i<4;i++)
    {
        odrv[i].velocityCL(vel[i],0);
    }
}


void Controller::odrv_pos_fix()
{
    for(int i=0; i<4;i++)
    {
        odrv[i].posControl(odrv[i].get_pos);
    }
}

void Controller::rmd_send()
{
    rmd_running=true;
    rmd_tmr.reset();
    while (rmd_running)
    {
        if (!rmd_sent && rmd_tmr.read_us() < CAN_INTERVAL)
        {
            rmd[0].sendCMD();
            rmd[1].sendCMD();
            rmd_sent = true;
        }
        else if (rmd_sent && rmd_tmr.read_us() > CAN_INTERVAL)
        {
            rmd[2].sendCMD();
            rmd[3].sendCMD();
            rmd_sent = false;
        }
        else if(rmd_tmr.read_us()> CAN_INTERVAL*2)
        {
            rmd_running=false;
        }
    }
}

void Controller::odrv_send()
{
    odrv_running=true;
    odrv_tmr.reset();
    while (odrv_running)
    {
        if (!odrv_sent && (odrv_tmr.read_us() < CAN_INTERVAL))
        {
            odrv[0].sendCMD();
            odrv[1].sendCMD();
            odrv_sent = true;
            //Serial.println(odrv_sent);
        }
        else if (odrv_sent && (odrv_tmr.read_us() > CAN_INTERVAL))
        {
            odrv[2].sendCMD();
            odrv[3].sendCMD();
            odrv_sent = false;
        }
        else if(odrv_tmr.read_us() > CAN_INTERVAL*2)
        {
            odrv_running=false;
        }
        
   
    }
}

void Controller::receive()
{
    for(int i=0;i<12;i++)
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


