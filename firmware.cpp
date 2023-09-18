#include "firmware.h"


/* library method */
Controller::Controller() : can(CAN_RX, CAN_TX)
{   
    rmd1 = new RMDmotor(1,can);
    rmd2 = new RMDmotor(2,can);
    rmd3 = new RMDmotor(3,can);
    rmd4 = new RMDmotor(4,can);
    odrv1 = new ODrive(48,can);
    odrv2 = new ODrive(49,can);
    odrv3 = new ODrive(50,can);
    odrv4 = new ODrive(51,can);
}

void Controller::init()
{
    while (!can.frequency(1000000));
    while (!can.mode(CAN::Normal));
    tmr.start();
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

void Controller::run()
{
    switch(rmd_state)
    {
        case START:
            rmd_reset();
            rmd_state=IDLE;
            break;
        case TORQUE:
        
    }
}

void Controller::rmd_reset()
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = rmd1->reset_zero();
            check[1] = rmd2->reset_zero();
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = rmd3->reset_zero();
            check[3] = rmd4->reset_zero();
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = rmd1->restart();
            check[1] = rmd2->restart();
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = rmd3->restart();
            check[3] = rmd4->restart();
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::rmd_torque(int16_t cmd[])
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = rmd1->torqueCL(cmd[0]);
            check[1] = rmd2->torqueCL(cmd[1]);
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = rmd3->torqueCL(cmd[2]);
            check[3] = rmd4->torqueCL(cmd[3]);
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::rmd_multi()
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = rmd1->read_Multi();
            check[1] = rmd2->read_Multi();
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = rmd3->read_Multi();
            check[3] = rmd4->read_Multi();
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::rmd_state()
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = rmd1->read_state2();
            check[1] = rmd2->read_state2();
            sent = true;
            secnt1++;
            secnt2++;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = rmd3->read_state2();
            check[3] = rmd4->read_state2();
            sent = false;
            secnt3++;
            secnt4++;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::receive_multi()
{
    for(int i=0;i<CAN_CNT;i++){
        if(can.read(rxmsg))
        {
            switch(rxmsg.id)
            {
                case 0x241:
                    memcpy(rmd1->receive_multi(),rxmsg.data,LEN);
                    break;
                case 0x242:
                    memcpy(rmd2->receive_multi(),rxmsg.data,LEN);
                    break;
                case 0x243:
                    memcpy(rmd3->receive_multi(),rxmsg.data,LEN);
                    break;
                case 0x244:
                    memcpy(rmd4->receive_multi(),rxmsg.data,LEN);
                    break;
            }
        }
    }
}


void Controller::receive_state()
{
    for(int i=0;i<CAN_CNT;i++){
        if(can.read(rxmsg))
        {
            switch(rxmsg.id)
            {
                case 0x241:
                    memcpy(rmd1->receive_state(),rxmsg.data,LEN);
                    recnt1++;
                    break;
                case 0x242:
                    memcpy(rmd2->receive_state(),rxmsg.data,LEN);
                    recnt2++;
                    break;
                case 0x243:
                    memcpy(rmd3->receive_state(),rxmsg.data,LEN);
                    recnt3++;
                    break;
                case 0x244:
                    memcpy(rmd4->receive_state(),rxmsg.data,LEN);
                    recnt4++;
                    break;
            }
        }
    }
}


void Controller::checking()
{
    if(recnt1==100000){
      txmsg.id=0x01;
      memcpy(txmsg.data,dummy,8);
      can.write(txmsg);
    }
    if(recnt2==100000){
      txmsg.id=0x02;
      memcpy(txmsg.data,dummy,8);
      can.write(txmsg);
    }
    if(recnt3==100000){
      txmsg.id=0x03;
      memcpy(txmsg.data,dummy,8);
      can.write(txmsg);
    }
    if(recnt4==100000){
      txmsg.id=0x04;
      memcpy(txmsg.data,dummy,8);
      can.write(txmsg);
    }
}


/*ODrive*/
void Controller::odrv_vel()
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = odrv1->velControl(command.odrv_cmd1,0);
            check[1] = odrv2->velControl(command.odrv_cmd2,0);
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = odrv3->velControl(command.odrv_cmd3,0);
            check[3] = odrv4->velControl(command.odrv_cmd4,0);
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::odrv_encoder()
{
    run=true;
    tmr.reset();
    while (run)
    {
        if (!sent && tmr.read_us() < CAN_INTERVAL)
        {
            check[0] = odrv1->read_encoder();
            check[1] = odrv2->read_encoder();
            sent = true;
        }
        else if (sent && tmr.read_us() > CAN_INTERVAL)
        {
            check[2] = odrv3->read_encoder();
            check[3] = odrv4->read_encoder();
            sent = false;
        }
        else if(tmr.read_us()> CAN_INTERVAL*2)
        {
            run=false;
        }
        else continue;
    }
}

void Controller::receve_encoder()
{
    for(int i=0;i<CAN_CNT;i++){
        if(can.read(rxmsg))
        {
            switch(rxmsg.id)
            {
                case 0x609:
                    memcpy(odrv1->receive_encoder(),rxmsg.data,LEN);
                    recnt1++;
                    break;
                case 0x629:
                    memcpy(odrv2->receive_encoder(),rxmsg.data,LEN);
                    recnt2++;
                    break;
                case 0x649:
                    memcpy(odrv3->receive_encoder(),rxmsg.data,LEN);
                    recnt3++;
                    break;
                case 0x669:
                    memcpy(odrv4->receive_encoder(),rxmsg.data,LEN);
                    recnt4++;
                    break;
            }
        }
    }
}

void Controller::cas()
{
    Serial.print("1  : ");
    Serial.println(odrv1->get_pos());
    Serial.print("2 : ");
    Serial.println(odrv2->get_pos());
    Serial.print("3 : ");
    Serial.println(odrv3->get_pos());
    Serial.print("4 : ");
    Serial.println(odrv4->get_pos());
}

Controller::~Controller() {
    delete rmd1;
    delete rmd2;
    delete rmd3;
    delete rmd4;
}