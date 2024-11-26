//
// term project 2024 controller code
// lab 004, team 1
// 

//
// main functions:
// forward, reverse, left, right control with tank turn
// control speed of drivetrain motors
// control hopper gate control
//

#define PRINT_SEND_STATUS
// #define PRINT_INCOMING                                

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros

// definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

// control data packet structure
typedef struct {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  uint32_t time;                                      // time packet sent
  int driveSpeed;                                     // variable for receiving motor speed
  int hopper;                                         // variable for hopper gate value
  bool left;                                          // variable for left button, either on or off
  bool right;                                         // variable for right button, either on or off
} __attribute__((packed)) esp_now_control_data_t;

// drive data packet structure
typedef struct {
  uint32_t time;                                      // time packet received
  int waterSpeed;                                     // pwm speed of water speed
  int colourTemp;                                     // colour temp value
  int spinDir;                                        // direction of sorting spin (0 = baseline, 1 = good, 2 = bad)
} __attribute__((packed)) esp_now_drive_data_t;

// button structure
struct Button {
  const int pin;                                      
  unsigned int numberPresses;                         
  unsigned int lastPressTime;                         
  bool pressed;                                       
  bool state;
  bool lastState;
};

// function declarations
void failReboot();
void ARDUINO_ISR_ATTR buttonISR(void* arg);

// constants
const int cStatusLED = 26;                            // GPIO pin of communication status LED
const int cDebounceDelay = 20;                        // button debounce delay in milliseconds
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop

// variables 
uint32_t lastTime = 0;                                // last time of motor control was updated
uint32_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Button buttonFwd = {33, 0, 0, false, true, true};     // forward, NO pushbutton on GPIO 14, low state when pressed
Button buttonRev = {26, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed

uint8_t receiverMacAddress[] = {0x88,0x13,0xBF,0x63,0x72,0x50};  
esp_now_control_data_t controlData;                   // data packet to send to drive system
esp_now_drive_data_t inData;                          // data packet from drive system

// added content
Button buttonLeft = {32, 0, 0, false, true, true};     // left, NO pushbutton on GPIO 13, low state when pressed
Button buttonRight = {27, 0, 0, false, true, true};    // right, NO pushbutton on GPIO 27, low state when pressed
int drivePotPin = 34;                                  // motor pot pin
int gatePotPin = 35;                                   // gate pot pin
int goodCount = 0;                                     // overall wanted bead counter
int badCount = 0;                                      // overall bad bead counter

// classes
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Network_Peer(const uint8_t *mac_addr, const uint8_t *lmk = NULL)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk) {}

    ~ESP_NOW_Network_Peer() {}
  bool begin() {
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
    return true;
    }
  bool send_message(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }
    return send(data, len);
    }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == 0)                                       
    {
      return;
    }
    memcpy(&inData, data, sizeof(inData));
    #ifdef PRINT_INCOMING
        Serial.printf("time: %d, colour temp: %d, spin dir: %d, collector speed: %d \n", inData.time, inData.colourTemp, inData.spinDir, inData.waterSpeed);
    #endif
    }
  
  void onSent(bool success) {
    if (success) {
    #ifdef PRINT_SEND_STATUS
          log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
          Serial.printf("dir: %d, left: %d, right: %d, drive: %d, hopper: %d \n", controlData.dir, controlData.left, controlData.right, controlData.driveSpeed, controlData.hopper); // troubleshooting
    #endif
        commsLossCount = 0;
      }
      else {
        digitalWrite(cStatusLED, 1);                      // turn on communication status LED
        commsLossCount++;
      }
    }
};

ESP_NOW_Network_Peer *peer;

void setup() {
  Serial.begin(115200);                               
  while (!Serial) {                                   
    delay(100);                                        
    }
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               
  while (!WiFi.STA.started()) {
    delay(100);                                       
    }
  Serial.print("MAC address for controller "); 
  Serial.println(WiFi.macAddress());                  
  
  // configure GPIO
  pinMode(cStatusLED, OUTPUT);                                      // configure GPIO for communication status LED as output
  pinMode(buttonFwd.pin, INPUT_PULLUP);                             // configure GPIO for forward button pin as an input with pullup resistor
  attachInterruptArg(buttonFwd.pin, buttonISR, &buttonFwd, CHANGE); // Configure forward pushbutton ISR to trigger on change
  pinMode(buttonRev.pin, INPUT_PULLUP);                             // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonRev.pin, buttonISR, &buttonRev, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  pinMode(buttonLeft.pin, INPUT_PULLUP);                                // left button
  attachInterruptArg(buttonLeft.pin, buttonISR, &buttonLeft, CHANGE);   // Configure left pushbutton ISR to trigger on change
  pinMode(buttonRight.pin, INPUT_PULLUP);                               // right button
  attachInterruptArg(buttonRight.pin, buttonISR, &buttonRight, CHANGE); // Configure right pushbutton ISR to trigger on change

  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
   }
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the drive peer\n");
    failReboot();
    }
    else {
      Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                   receiverMacAddress[2], receiverMacAddress[3], 
                                                                   receiverMacAddress[4], receiverMacAddress[5]);
    }
  memset(&inData, 0, sizeof(inData));                 // clear drive data
  memset(&controlData, 0, sizeof(controlData));       // clear controller data
}

void loop() {

  uint32_t curTime = micros();                        
  if (curTime - lastTime > 10000) {                  
    lastTime = curTime;
    controlData.time = curTime;                       
    if (commsLossCount > cMaxDroppedPackets) {
        controlData.dir = 0;
    }
    if (peer->send_message((const uint8_t *) &controlData, sizeof(controlData))) {
        digitalWrite(cStatusLED, 1);
      }
      else {
        digitalWrite(cStatusLED, 0);
    }   

    // drivetrain speed control
    int driveSpeed = analogRead(drivePotPin);                       // drive pot value read
    controlData.driveSpeed = map(driveSpeed, 0, 4095, 0, 14);       // scale raw pot value into dc range, send over

    // hopper gate control
    int gateVal = analogRead(gatePotPin);                           // gate pot value read
    controlData.hopper = map(gateVal, 0, 4095, 85, 180);            // scale raw pot value into degrees, range from 90 to 180 degrees

    // forward and reverse button operation
    if (!buttonFwd.state) {                           // forward pushbutton pressed
      controlData.dir = 1;
      }
      else if (!buttonRev.state) {                    // reverse pushbutton pressed
      controlData.dir = -1;
      }
      else {                                          // no input, stop
      controlData.dir = 0;
    }
    
    // left and right button operation
    if (!buttonLeft.state) {                          // left button
      controlData.left = 1;
      } else {                                         
      controlData.left = 0;
      }
      if (!buttonRight.state) {                       // right button
      controlData.right = 1;
      } else {             
      controlData.right = 0;
    }
  }
  // Serial.printf("dir: %d, left: %d, right: %d, drive: %d, hopper: %d \n", controlData.dir, controlData.left, controlData.right, controlData.driveSpeed, controlData.hopper);
}

void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              
  uint32_t pressTime = millis();
  s->state = digitalRead(s->pin);
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            
    s->pressed = true;
  }
  s->lastPressTime = pressTime;                       
  s->lastState = s->state;
}