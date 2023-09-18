#include "firmware.h"
Controller control;
Timer mt;


void setup() {
  // put your setup code here, to run once:
  control.init();
  Serial.begin(115200);
  mt.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  control.run_vel();
  
  //control.receive_encoder();
  if(mt.read_ms()>1000)
  {
    mt.reset();
    control.cas();
  }
}
