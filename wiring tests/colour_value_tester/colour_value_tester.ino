#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

bool tcsFlag = 0;                                     
const int cTCSLED = 23;                               // GPIO pin for LED on TCS34725
uint16_t r, g, b, c;   
int lsTime = 0;     
int colourTemp = 0; 

void setup() {
  Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  while (!Serial) {                                   // wait for Serial to start
    delay(10);                                        // okay to delay during setup
    }

  if (tcs.begin()) {
  Serial.printf("Found TCS34725 colour sensor\n");
  tcsFlag = true;
  digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } else {
  Serial.printf("No TCS34725 found ... check your connections\n");
  tcsFlag = false;
  }
}

void loop() {
  uint32_t cTime = millis();                       // capture current time in microseconds
  if (cTime - lsTime > 1000) {                     // wait ~1s
    lsTime = cTime;
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    colourTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    Serial.printf("colour temp: %d, r: %d, g: %d, b: %d, c: %d\n", colourTemp, r, g, b, c);
    if ( c <= 70 ) { // baseline values for servo to stay in middle, either after scanning slide face or open air
      Serial.printf(" baseline ");
      spinDir = 0;
      } 
      else if ( (colourTemp >= 4000) && (colourTemp <= 5000) && (c >= 80) && (r <= 80)) { // good or wanted values, spin left
      Serial.printf(" good");
      }
      else if ( colourTemp = 0 ) { // restart esp32 if colour sensor stops working, unknown cause
        failReboot();
      } 
      else { // any other value is bad, spin to the back
      Serial.printf(" bad ");
    }
  }
}
