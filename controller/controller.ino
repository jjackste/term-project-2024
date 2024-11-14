//
// term project 2024 controller code [drivetrain]
// adapted from mme4487 lab 4 controller code
// lab 004, team 1
// 

//
// main functions:
// forward, reverse, left, right control with tank turn
// control speed of drivetrain motors
//

#define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

// Structs
// Control data packet structure
typedef struct {
  int dir;                                            //drive direction: 1 = forward, -1 = reverse, 0 = stop
  uint32_t time;                                      //time packet sent
  int driveSpeed;                                     //motor speed
  bool left;                                          //is left button pressed?
  bool right;                                         //is right button pressed?
  int waterSpeed;
} __attribute__((packed)) esp_now_control_data_t;

// Drive data packet structure
typedef struct {
  uint32_t time;                                      // time packet received
  int colourTemp;                                     // colour score value
  int spinDir;                                        // direction of sorting spin
} __attribute__((packed)) esp_now_drive_data_t;

// Button structure
struct Button {
  const int pin;                                      // GPIO pin for button
  unsigned int numberPresses;                         // counter for number of button presses
  unsigned int lastPressTime;                         // time of last button press in ms
  bool pressed;                                       // flag for button press event
  bool state;                                         // current state of button; 0 = pressed; 1 = unpressed
  bool lastState;                                     // last state of button
};

// Function declarations
void failReboot();
void ARDUINO_ISR_ATTR buttonISR(void* arg);

// Constants
const int cStatusLED = 26;                            // GPIO pin of communication status LED
const int cDebounceDelay = 20;                        // button debounce delay in milliseconds
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop

// Variables 
uint32_t lastTime = 0;                                // last time of motor control was updated
uint32_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Button buttonFwd = {12, 0, 0, false, true, true};     // forward, NO pushbutton on GPIO 14, low state when pressed
Button buttonRev = {14, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed

// REPLACE WITH MAC ADDRESS OF YOUR DRIVE ESP32
uint8_t receiverMacAddress[] = {0x88,0x13,0xBF,0x63,0x72,0x50};  
esp_now_control_data_t controlData;                   // data packet to send to drive system
esp_now_drive_data_t inData;                          // data packet from drive system

// added content
Button buttonLeft = {27, 0, 0, false, true, true};     // left, NO pushbutton on GPIO 13, low state when pressed
Button buttonRight = {33, 0, 0, false, true, true};    // right, NO pushbutton on GPIO 27, low state when pressed
int motorPotPin = 34;                                  // motor pot pin

// Classes
class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Network_Peer(const uint8_t *mac_addr, const uint8_t *lmk = NULL)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk) {}

  ~ESP_NOW_Network_Peer() {}

  bool begin() {
    // Assumes that the ESP-NOW protocol is already initialized
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
    // Call the parent class method to send the data
    return send(data, len);
  }

  // callback function for when data is received
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == 0)                                       // if empty packet
    {
      return;                                           // return
    }
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller
  #ifdef PRINT_INCOMING
      Serial.printf("%d\n", inData.time);
  #endif
  }
  
  // callback function for when data is sent
  void onSent(bool success) {
    if (success) {
  #ifdef PRINT_SEND_STATUS
        log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
        Serial.printf("%d, %d, %d \n", controlData.dir, controlData.left, controlData.right); // troubleshooting
  #endif
      commsLossCount = 0;
    }
    else {
      digitalWrite(cStatusLED, 1);                      // turn on communication status LED
      commsLossCount++;
    }
  }
};

// Peers
ESP_NOW_Network_Peer *peer;

void setup() {
  Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  while (!Serial) {                                   // wait for Serial to start
    delay(10);                                        // okay to delay during setup
  }
  WiFi.mode(WIFI_STA);                                // use WiFi in station mode
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               // set WiFi channel to use with peer
  while (!WiFi.STA.started()) {                       // wait for WiFi to start
    delay(100);                                       // okay to delay during setup
  }
  Serial.print("MAC address for controller "); 
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  
  // Configure GPIO
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output
  pinMode(buttonFwd.pin, INPUT_PULLUP);               // configure GPIO for forward button pin as an input with pullup resistor
  attachInterruptArg(buttonFwd.pin, buttonISR, &buttonFwd, CHANGE); // Configure forward pushbutton ISR to trigger on change
  pinMode(buttonRev.pin, INPUT_PULLUP);               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonRev.pin, buttonISR, &buttonRev, CHANGE); // Configure reverse pushbutton ISR to trigger on change

  pinMode(buttonLeft.pin, INPUT_PULLUP);                          //left button
  attachInterruptArg(buttonLeft.pin, buttonISR, &buttonLeft, CHANGE); // Configure left pushbutton ISR to trigger on change
  pinMode(buttonRight.pin, INPUT_PULLUP);                          //right button
  attachInterruptArg(buttonRight.pin, buttonISR, &buttonRight, CHANGE); // Configure right pushbutton ISR to trigger on change

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
  }

  // add drive as peer
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
  uint32_t curTime = micros();                        // capture current time in microseconds

  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    lastTime = curTime;
    controlData.time = curTime;                       // update transmission time
    
    // if drive appears disconnected, update control signal to stop before sending
    if (commsLossCount > cMaxDroppedPackets) {
        controlData.dir = 0;
      }
    // send control signal to drive
    if (peer->send_message((const uint8_t *) &controlData, sizeof(controlData))) {
        digitalWrite(cStatusLED, 1);                    // if successful, turn on communucation status LED
      }
      else {
        digitalWrite(cStatusLED, 0);                    // otherwise, turn off communication status LED
      }

    // speed control
    int driveSpeed = analogRead(motorPotPin);                       // pot value sent as a variable in the structure
    controlData.motorSpeed = map(driveSpeed, 0, 4095, 0, 14);       // scale raw pot value into servo range 


    //forward and reverse button operation
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
    if (!buttonRight.state) {                         // right button
      controlData.right = 1;
    } else {             
      controlData.right = 0;
    }
  }
}

// function to reboot the device
void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, allowing multiple
// instances of the buttonISR to be created (1 per button)
// implements software debounce and tracks button state
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              // cast pointer to static structure

  uint32_t pressTime = millis();                      // capture current time
  s->state = digitalRead(s->pin);                     // capture state of button
  // if button has been pressed and sufficient time has elapsed
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            // increment button press counter
    s->pressed = true;                                // set flag for "valid" button press
  }
  s->lastPressTime = pressTime;                       // update time of last state change
  s->lastState = s->state;                            // save last state
}