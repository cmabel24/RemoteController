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
#ifdef _DEBUG_
  _PL("Left");
  _PP("  Fault: "); _PL(message.faultLS);
  _PP("  Amps: "); _PL(message.motorAmpsLS);
  _PL("Right");
  _PP("  Fault: "); _PL(message.faultRS);
  _PP("  Amps: "); _PL(message.motorAmpsRS);
#endif /* _DEBUG */
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


// An example of the interrupts
// void setup() {
//    // Set up the generic clock (GCLK4) used to clock timers
//   REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
//                     GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
//   while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

//   REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
//                      GCLK_GENCTRL_GENEN |         // Enable GCLK4
//                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
//                      GCLK_GENCTRL_ID(4);          // Select GCLK4
//   while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

//   // Feed GCLK4 to TC4 and TC5
//   REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
//                      GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
//                      GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
//   while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

//   REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;           // Set the counter to 8-bit mode
//   while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

//   REG_TC4_COUNT8_CC0 = 0x55;                      // Set the TC4 CC0 register to some arbitary value
//   while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
//   REG_TC4_COUNT8_CC1 = 0xAA;                      // Set the TC4 CC1 register to some arbitary value
//   while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
//   REG_TC4_COUNT8_PER = 0xFF;                      // Set the PER (period) register to its maximum value
//   while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

//   //NVIC_DisableIRQ(TC4_IRQn);
//   //NVIC_ClearPendingIRQ(TC4_IRQn);
//   NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
//   NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

//   REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags
//   REG_TC4_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 | TC_INTENSET_OVF;     // Enable TC4 interrupts
//   // REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 | TC_INTENCLR_OVF;     // Disable TC4 interrupts
 
//   REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV64 |     // Set prescaler to 64, 16MHz/64 = 256kHz
//                    TC_CTRLA_ENABLE;               // Enable TC4
//   while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
// }


// void TC4_Handler()                              // Interrupt Service Routine (ISR) for timer TC4
// {     
//   // Check for overflow (OVF) interrupt
//   if (TC4->COUNT8.INTFLAG.bit.OVF && TC4->COUNT8.INTENSET.bit.OVF)             
//   {
//     // Put your timer overflow (OVF) code here:     
//     // ...
   
//     REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
//   }

//   // Check for match counter 0 (MC0) interrupt
//   if (TC4->COUNT8.INTFLAG.bit.MC0 && TC4->COUNT8.INTENSET.bit.MC0)             
//   {
//     // Put your counter compare 0 (CC0) code here:
//     // ...
   
//     REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
//   }

//   // Check for match counter 1 (MC1) interrupt
//   if (TC4->COUNT8.INTFLAG.bit.MC1 && TC4->COUNT8.INTENSET.bit.MC1)           
//   {
//     // Put your counter compare 1 (CC1) code here:
//     // ...
   
//     REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
//   }
// }


#endif /* CONTROLLER_H */ 
