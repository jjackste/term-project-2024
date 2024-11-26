#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);
void failReboot();
bool tcsFlag = 0;
const int cTCSLED = 23;  // GPIO pin for LED on TCS34725
uint16_t r, g, b, c;
int lsTime = 0;
int lsTime2 = 0;
int colourTemp = 0;
int servo = 25;
int pot = 26;
const int cMinDutyCycle = 1700;  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;  // duty cycle for 180 degrees

void setup() {
  Serial.begin(115200);  // standard baud rate for ESP32 serial monitor
  while (!Serial) {      // wait for Serial to start
    delay(10);           // okay to delay during setup
  }

  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  ledcAttach(servo, 50, 16);
  ledcWrite(servo, 90);
}

void loop() {
  uint32_t cTime = millis();    // capture current time in microseconds
  if (cTime - lsTime > 1000) {  // wait ~1s
    lsTime = cTime;
    digitalWrite(cTCSLED, 1);
    tcs.getRawData(&r, &g, &b, &c);  // get raw RGBC values
    digitalWrite(cTCSLED, 0);

    colourTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    Serial.printf("colour temp: %d, r: %d, g: %d, b: %d, c: %d,", colourTemp, r, g, b, c);
   
    if ((c >= 1) && (c <= 210)) {  // baseline values for servo to stay in middle, either after scanning slide face or open air
      Serial.printf(" baseline \n");
      ledcWrite(servo, 90);
    } else if ((colourTemp <= 4700) && (colourTemp >= 4000) && (r <= 300) && (r >= 100) && (g <= 300) && (g >= 95) && (b <= 200) && (b >= 75) && (c >= 275) && (c <= 1484)) {
      Serial.printf(" good \n"); 
      ledcWrite(servo, 0);
    } else if (colourTemp = 0) {  // restart esp32 if colour sensor stops working, unknown cause
      failReboot();
    } else {  // any other value is bad, spin to the back
      Serial.printf(" bad \n");
      ledcWrite(servo, 180);
    }
  }
}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
  return dutyCycle;
}

void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}
