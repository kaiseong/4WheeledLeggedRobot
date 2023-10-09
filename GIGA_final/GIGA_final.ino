#include "firmware.h"

#define FLAG_CAN  (1UL<<0)


Controller control;
Keyboard keyb;
HostSerial ser;
int zlow=32;
int32_t zs[37] = { 264, 272, 278, 283, 288, 292, 295, 299, 302, 306, 309, 312, 315, 318, 320, 323, 326, 329, 331, 334, 336, 339, 342, 345, 347, 349, 353, 357, 0, 3, 5, 8, 12, 20, 25, 32, 47 };
double v=0;
double w=0;
int pitch=0;
int roll=0;
int rr;
int rl;
int fr;
int fl;
//int zrl;
//int zfr;
double z;

using namespace rtos;
Thread th1;
int cnt=0;
bool drive_mode=false;
int32_t pos[4]={0,0,0,0};
float vela[4]={0,0,0,0};

void task();

void setup() {
  // put your setup code here, to run once:
  control.init();
  Serial.begin(115200);
  //while (!Serial);
  control.rmd_reset();
  controle.odrv_encoder();
  th1.start(&task);
  //th2.start(&task2);
  pinMode(PA_15, OUTPUT);
  keyb.begin();
  ser.begin();
  v=0;
  w=0;
  pitch=0;
  roll=0;
  z=32;


}

void loop() {
  // put your main code here, to run repeatedly:
  //// put your main code here, to run repeatedly:
  //    //Serial.println(key);
  //  //Serial.print('v:');
  Serial.println(v);
  ////Serial.print('w:');
  Serial.println(w);
  ////Serial.println(pitch);
  ////Serial.println(roll);
  Serial.println(z);
  if (keyb.available()) {
    auto _key = keyb.read();
    char key=keyb.getAscii(_key);
    switch(key){
      case 'w':
        v=v+0.1;
        break;
      case 'a':
        w=w+0.01;
        break;
      case 'd':
        w=w-0.01;
        break;
      case 'x':
        v=v-0.1;
        break;
      case 's':
        v=0;
        w=0;
        break;
      case 'q':
        if((z)>34 && (z)<64){
          pitch=pitch-2;
          //zfr=zfr-1;
        }
        else
          pitch=pitch;
        break;
      case 'e':
        if((z)>34 && (z)<64){
          pitch=pitch+2;
          //zfr=zfr+1;
        }
        else
          pitch=pitch;
        break;
      case 'z':
        if((z)>34 && (z)<64){
          roll=roll-2;
          //zrl=zrl-1;
        }
        else
        roll=roll;
        break;
      case 'c':
        if((z)>34 && (z)<64){
          roll=roll+2;
          //zrl=zrl+1;
        }
        else
          roll=roll;
        break;
      case 'r':
        if((z)>32)
          z=z-1;
        //else
          //z=32;
        break;
      case 't':
        if((z)<68)
          z=z+1;
        //else
          //z=68;
        break;
      default:
        v=v;
        w=w;
        pitch=pitch;
        roll=roll;
        z=z;
        break;
    }

  }
  int zfr=(z-zlow);
  int zfl=(z-zlow);
  int zrr=(z-zlow);
  int zrl=(z-zlow);
  double l_vel=(v-(w*0.61/2))/3.14/0.13;
  double r_vel=(v+(w*0.61/2))/3.14/0.13;
  if(z>=60){
  rr=zs[zrr]+360-264;
  rl=zs[zrl]+360-264;
  fr=zs[zfr]+360-264;
  fl=zs[zfl]+360-264;
  }
  else{
  rr=zs[zrr]-264;
  rl=zs[zrl]-264;
  fr=zs[zfr]-264;
  fl=zs[zfl]-264;
  }  
  pos[0]=fl*100;
  pos[1]=fr*-100;
  pos[2]=rl*100;
  pos[3]=rr*-100;
  vela[0]=l_vel;
  vela[1]=r_vel;
  vela[2]=l_vel;
  vela[3]=r_vel;
  control.rmd_pos(pos);
  control.odrv_pos_fix();
}

void task()
{
  while(true)
  {
    control.rmd_send();
    cnt++;
    if(cnt>70)
    {
      control.odrv_send();
      cnt=0;
    }
    rtos::ThisThread::sleep_for(1);
  }
}
