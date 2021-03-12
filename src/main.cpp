#define _DEBUG_

#include "controller.h"

void setup() {  
  analogReadResolution(12);
  Oled.init();
  Controller.init();
  LocalSerial.init();
  // ts.setHighPriorityScheduler(&hpr); 
}

void loop() {
  if (Controller.test()) 
    tInterrupt.restart();
  if (Serial1.available())
    tRead.restart();
  ts.execute();
}
