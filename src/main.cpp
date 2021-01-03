#define _DEBUG_

#include "controller.h"

void setup() {  
  analogReadResolution(12);
  Controller.init();
  LocalSerial.init();
}

void loop() {
  if (Serial1.available())
    tRead.restart();
  ts.execute();
}
