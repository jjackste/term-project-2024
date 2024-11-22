#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);


const long cMinDutyCycle = 1650;                 // duty cycle for 0 degrees (ad2ust for motor if necessary)
const long cMaxDutyCycle = 8175;
const int sorterPin = 5;
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

  ledcAttach(sorterPin, 50, 16);                    // setup sorter pin for 50 Hz, 16-bit resolution

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
  if (cTime - lsTime > 500) {                   // wait ~10 ms
    lsTime = cTime;
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    colourTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    Serial.printf("colour temp: %d \n", colourTemp);
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");

    if( (colourTemp >= 3800) && (colourTemp <= 4500)  && (r <= 30) && (g <= 20) && (b <= 20) ) { // baseline
      ledcWrite(sorterPin, degreesToDutyCycle(90));
      Serial.printf("baseline \n");
      } 
      else if ( (colourTemp >= 4000) && (colourTemp <= 5000) && (c >= 80) && (r <= 80)) { // good
      ledcWrite(sorterPin, degreesToDutyCycle(0));
      Serial.printf("good \n");
      } 
      else { // any other value if bad
      ledcWrite(sorterPin, degreesToDutyCycle(180));
      Serial.printf("bad \n");
    }
  }

}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle
#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0015259;              // dutyCycle / 65535 * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif
  return dutyCycle;
}