#ifndef OLED_H
#define OLED_H

#include <Wire.h>  // Include Wire if you're using I2C
#include <SPI.h>  // Include SPI if you're using SPI
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_CS    10 // Connect CS to pin 10
#define PIN_RESET 11 // Connect RST to pin 11
#define PIN_DC    12 // Connect DC to pin 12
#define DC_JUMPER 0  // Set to either 0 (default) or 1 based on jumper, matching the value of the DC Jumper


class Oled {
  private:
  void printTitle(String title, int font);
  MicroOLED oled;

  public:
  Oled();
  void init();
  void loading();
  void displayData(bool faultLS, bool faultRS, uint32_t motorAmpsLS, uint32_t motorAmpsRS, uint32_t vBatt);
} Oled;


Oled::Oled () : oled(PIN_RESET, PIN_DC, PIN_CS) {} //SPI configuration

void Oled::init() {
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000);
  oled.clear(PAGE); // Clear the buffer.
  loading();
  delay(1000);
}

void Oled::printTitle(String title, int font) {
  int middleX = oled.getLCDWidth() / 2;
  int middleY = oled.getLCDHeight() / 2;

  oled.clear(PAGE);
  oled.setFontType(font);
  // Try to set the cursor in the middle of the screen
  oled.setCursor(middleX - (oled.getFontWidth() * (title.length() / 2)),
  middleY - (oled.getFontHeight() / 2));
  // Print the title:
  oled.print(title);
  oled.display();
  delay(1500);
  oled.clear(PAGE);
}

void Oled::loading() {
  int middleX = oled.getLCDWidth() / 2;
  int middleY = oled.getLCDHeight() / 2;
  int xEnd, yEnd;
  int lineWidth = min(middleX, middleY);

  printTitle("CoolRC", 1);

  for (int i = 0; i < 3; i++)
  {
    for (int deg = 0; deg < 360; deg += 15)
    {
      xEnd = lineWidth * cos(deg * PI / 180.0);
      yEnd = lineWidth * sin(deg * PI / 180.0);

      oled.line(middleX, middleY, middleX + xEnd, middleY + yEnd);
      oled.display();
      delay(10);
    }
    for (int deg = 0; deg < 360; deg += 15)
    {
      xEnd = lineWidth * cos(deg * PI / 180.0);
      yEnd = lineWidth * sin(deg * PI / 180.0);

      oled.line(middleX, middleY, middleX + xEnd, middleY + yEnd, BLACK, NORM);
      oled.display();
      delay(10);
    }
  }
}

void Oled::displayData(bool faultLS, bool faultRS, uint32_t motorAmpsLS, uint32_t motorAmpsRS, uint32_t vBatt) {
  oled.clear(PAGE);    
  oled.setCursor(0, 0);       
  oled.setFontType(0);  
  oled.print("LS: ");      
  oled.setFontType(2);
  if (faultLS)
    oled.print("Fault"); 
  else 
    oled.print(motorAmpsLS); 
  oled.setCursor(0, 16);    
  oled.setFontType(0);        
  oled.print("RS: ");
  oled.setFontType(2);
  if (faultRS)
    oled.print("Fault"); 
  else 
    oled.print(motorAmpsRS); 
  oled.setCursor(0, 32);
  oled.setFontType(0);
  oled.print("Vrover: ");
  oled.setFontType(2);
  oled.print(vBatt);
  oled.display();
}

#endif //OLED_H
