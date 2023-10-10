#include "firmware.h"

#define FLAG_MAIN  (1UL<<0)
#define FLAG_SUB   (1UL<<1)
#define FLAG_IMU   (1UL<<2)

EventFlags ef;

Controller control;

using namespace rtos;
Thread th1(osPriorityRealtime);
Thread th2(osPriorityRealtime);
Thread th3(osPriorityRealtime);
Ticker tic;
uint32_t tickcnt=0;

float Gain_Kp[4] = {0,};
float Gain_Ki[4] = {0,};
int32_t pos[4]={1,2,3,4};
float vel[4]={5,6,7,8};

void task_main();
void task_sub();
void task_imu();
void setFlag();
void test();

void setup() {
  // put your setup code here, to run once:
  control.init();
  Serial.begin(115200);
  //while (!Serial);
  control.rmd_reset();

  tic.attach_us(&setFlag,CAN_INTERVAL);
  th1.start(&task_main);
  th2.start(&task_sub);
  th3.start(&task_imu);


}

void loop() {
}

void task_main()
{
  while(true)
  {
    ef.wait_any(FLAG_MAIN);
    control.run(pos,vel,Gain_Kp, Gain_Ki);

  }
}

void task_sub(){
  while(true){
    ef.wait_any(FLAG_SUB);
    control.receive();
    test();
  }
}

void task_imu(){
  while(true){
    ef.wait_any(FLAG_IMU);
    
    
  }
  
}


void setFlag(){
  ef.set(FLAG_MAIN);
  if(tickcnt % 4 == 0){ 
    ef.set(FLAG_IMU);
    ef.set(FLAG_SUB);
  }
  tickcnt++;
}

void test(){

}

