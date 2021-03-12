#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <pb_encode.h>
#include <pb_decode.h>
#include <pb.h>
#ifdef _TEST_
#include <unity.h>
#endif // _TEST_ 

#include "serialCommunications.pb.h"
#include "scheduler.h"
#include "oled.h"

#define _TANK_


/*
*          EIC  ZERO pins
*            0    11
*            1    13
*            2    10 a0 a5
*            3    12
*            4     6 a3
*            5     7 a4
*            6     8  sda(20)
*            7     9  scl(21)
*            8    a1
*            9     3 a2
*           10     1 MOSI(23)
*           11     0  SCK(24)
*           12     MISO(22)
*           13
*           14     2
*           15     5
*/


// Task Scheduler Callbacks
void sendCommand();
void readByte();
void decodeTellemetry();
void displayTellemetry();

// Tasks
Task tInterrupt(TASK_IMMEDIATE, TASK_ONCE, &sendCommand, &ts);
Task tRead(TASK_IMMEDIATE, TASK_ONCE, &readByte, &ts);
Task tDecode(TASK_IMMEDIATE, TASK_ONCE, &decodeTellemetry, &ts);
Task tDisplay(TASK_IMMEDIATE, TASK_ONCE, &displayTellemetry, &ts);
Task tKeepAlive(500, TASK_FOREVER, &sendCommand, &ts, true);



class Controller {
  private:
    ControllerCommands message = ControllerCommands_init_default;
    ControllerCommands messageLast = ControllerCommands_init_default;
    uint8_t buffer[ControllerCommands_size];
    int analog(int);
    void read();

#ifdef _TANK_
    // Interrupt mapping
    int iA0 = 2; // joystick right +up -down
    int iA1 = 8; // joystick right +left -right 
    int iA2 = 9; // joystick left +left -right 
    int iA3 = 4; // joystick left +up -down
    int iD2 = 14;   // right thumb press
    int iD3 = 9;   // right finger
    int iD5 = 15;   // left thumb press
    int iD6 = 4;   // left finger
#else
    // Interrupt mapping
    int iA2 = 9; // joystick left +left -right 
    int iA3 = 4; // joystick left +up -down
    int iD2 = 14;   // "X"
    int iD3 = 9;   // right finger
    char iD4 = "NMI";   // "square"
    int iD5 = 15;   // left thumb press
    int iD6 = 4;   // left finger
    int iD8 = 6;  // "triangle"
    int iD9 = 7;  // "circle"
#endif /* _TANK_ */
  
  public:
    void init();
    void send();
    bool test();

} Controller;



class Rover {
  private:
    RoverTellemetry message = RoverTellemetry_init_default;
    uint8_t buffer[RoverTellemetry_size];
    size_t rlen;
    int idx;

  public:
    void read();
    void display();
    bool decode();
} Rover;



class LocalSerial {
  public:
    void init();
    void flush();
} LocalSerial ;



// Callback definitions
void sendCommand() {Controller.send();}
void displayTellemetry() {Rover.display();}
void readByte() {Rover.read();}
void decodeTellemetry() {
  if (!Rover.decode())
    tDisplay.restart();
}


// Controller Class Methods
int Controller::analog(int pin) {
  uint16_t stick = analogRead(pin);
  int val;
  if (stick < 2096 && stick > 1999)
    val = 0;
  else
    val = (stick - 2000)/5;
  if (val > 400)
    val = 400;
  if (val < -400)
    val = -400;
  return val;
}

void Controller::read() {
#ifdef _TANK_
  message.a0 = analog(A0);
  message.a1 = analog(A1);
  message.a2 = analog(A2);
  message.a3 = analog(A3);
  message.d2 = digitalRead(2);
  message.d3 = digitalRead(3);
  message.d5 = digitalRead(5);
  message.d6 = digitalRead(6);
#else
  message.a2 = analog(A2);     // joystick left +left -right 
  message.a3 = analog(A3);     // joystick left +up -down
  message.d2 = digitalRead(2); // "X"
  message.d3 = digitalRead(3); // right finger
  message.d4 = digitalRead(4); // "square"
  message.d5 = digitalRead(5); // left thumb press
  message.d6 = digitalRead(6); // left finger
  message.d8 = digitalRead(8); // "triangle"
  message.d9 = digitalRead(9); // "circle"
#endif /* _TANK_ */
}

void Controller::init() {
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
}

void Controller::send() {
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  read();
  bool status = pb_encode(&stream, ControllerCommands_fields, &message);
  // size_t message_length = stream.bytes_written;
  
  if (!status) {
#ifdef _DEBUG_
  char buff[128];
  sprintf (buff, "Encoding failed: %s\n", PB_GET_ERROR(&stream));
  _PP(buff);
#endif /* _DEBUG_ */
  } else {
    Serial1.write(buffer, sizeof(buffer));
    Serial1.write('\r');
    }
}

bool Controller::test() {
  if (message.a3 == messageLast.a3 && message.a0 == messageLast.a0)
    return false;
  messageLast = message;
  return true;
}


//Rover Class Methods
void Rover::read() {
  uint8_t recievedByte = Serial1.read();
  if (recievedByte == '\r') {
    idx = 0;
    tDecode.restart();
  } else if (idx < RoverTellemetry_size) {
    buffer[idx] = recievedByte;
    idx++;
  }
}

bool Rover::decode() {
  pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));
  bool status = pb_decode(&stream, RoverTellemetry_fields, &message);
  
#ifdef _DEBUG_ 
  if (status){
    char buff[128];
    sprintf (buff, "Decoding failed: %s\n", PB_GET_ERROR(&stream));
    _PP(buff);
  }
#endif /* _DEBUG_ */

  return status;
}

void Rover::display() {
  Oled.displayData(message.faultLS, message.faultRS, message.motorAmpsLS, message.motorAmpsRS, message.vBatt);
}


// Local serial methods
void LocalSerial::init() {
#ifdef _DEBUG_
  SerialUSB.begin(115200);
#endif /* _DEBUG_ */

  // Clear the serial buffer
  flush();
  delay(10);
  flush();
  Serial1.begin(115200);
}

void LocalSerial::flush(){
  while(Serial1.available())
    Serial1.read();
}

#endif /* CONTROLLER_H */ 
