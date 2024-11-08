// 
// MME 4487 Lab 4 Drive
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Michael Naish
//  Date:     2024 10 07
//

// #define SERIAL_STUDIO                                 // print formatted string, that can be captured and parsed by Serial-Studio

// #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  // For the MAC2STR and MACSTR macros
#include "Adafruit_TCS34725.h"

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 // Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4                         // Channel to be used by the ESP-NOW protocol

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X); //setup adafruit sesnor

// Control data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  uint32_t time;                                      // time packet sent
  int speed;                                          // variabel for receiving motor speed
  bool left;                                          // variable for left button, either on or off
  bool right;                                         // variable for right button, either on or off
} __attribute__((packed)) esp_now_control_data_t;

// Drive data packet structure
// The attribute "packed" ensures that the struct is not padded (all data is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
typedef struct {
  uint32_t time;                                      // time packet sent
  int colorTemp;                                      // variable to send out color temp data
} __attribute__((packed)) esp_now_drive_data_t;

// Encoder structure
struct Encoder {
  const int chanA;                                    // GPIO pin for encoder channel A
  const int chanB;                                    // GPIO pin for encoder channel B
  int32_t pos;                                        // current encoder position
};

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void failReboot();
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cStatusLED = 27;                            // GPIO pin of communication status LED
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cNumMotors = 2;                             // Number of DC motors
const int cIN1Pin[] = {17, 19};                       // GPIO pin(s) for INT1
const int cIN2Pin[] = {16, 18};                       // GPIO pin(s) for INT2
const int cPWMRes = 8;                                // bit resolution for PWM
const int cMinPWM = 0;                                // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;              // PWM value for maximum speed
const int cPWMFreq = 20000;                           // frequency of PWM signal
const int cCountsRev = 1096;                          // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                   // maximum encoder counts/sec
const int cMaxChange = 14;                            // maximum increment in counts/cycle
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const float cKp = 1.5;                                // proportional gain for PID
const float cKi = 0.2;                                // integral gain for PID
const float cKd = 0.8;                                // derivative gain for PID

// Variables
uint32_t lastHeartbeat = 0;                           // time of last heartbeat state change
uint32_t lastTime = 0;                                // last time of motor control was updated
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position
                     {32, 33, 0}};                    // encoder 1 on GPIO 32 and 33, 0 position
int32_t target[] = {0, 0};                            // target encoder count for motor
int32_t lastEncoder[] = {0, 0};                       // encoder count at last control cycle
float targetF[] = {0.0, 0.0};                         // target for motor as float

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = {0xAC,0x15,0x18,0xD6,0x81,0x34};  // MAC address of controller 00:01:02:03:04:05
esp_now_control_data_t inData;                        // control data packet from controller
esp_now_drive_data_t driveData;                       // data packet to send to controller

// Classes
//
// The ESP_NOW_NetworkPeer class inherits from ESP_NOW_Peer and implements the _onReceive and _onSent methods.
// For more information about the ESP_NOW_Peer class, see the ESP_NOW_Peer class in the ESP32_NOW.h file.
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
    Serial.printf("%d, %d, %d, %d, %d\n", inData.dir, inData.speed, inData.left, inData.right, inData.time);
#endif
  }
  
  // callback function for when data is sent
  void onSent(bool success) {
    if (success) {
#ifdef PRINT_SEND_STATUS
      log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
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

  Serial.print("MAC address for drive "); 
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output

  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);        // setup INT1 GPIO PWM channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);        // setup INT2 GPIO PWM channel
    pinMode(encoder[k].chanA, INPUT);                 // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                 // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
  }
  
  // add controller as peer
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the controller peer\n");
    failReboot();
  }
  else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                 receiverMacAddress[2], receiverMacAddress[3], 
                                                                 receiverMacAddress[4], receiverMacAddress[5]);
  }
  memset(&inData, 0, sizeof(inData));                 // clear controller data
  memset(&driveData, 0, sizeof(driveData));           // clear drive data
}

void loop() {
  float deltaT = 0;                                   // time interval
  int32_t pos[] = {0, 0};                             // current motor positions
  int32_t e[] = {0, 0};                               // position error
  float velEncoder[] = {0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0};                         // change in position for set speed
  float ePrev[] = {0, 0};                             // previous position error
  float dedt[] = {0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0};                         // integral of error 
  float u[] = {0, 0};                                 // PID control signal
  int pwm[] = {0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1};                                 // direction that motor should turn
  
  uint16_t r, g, b, c, colorTemp;                     // variables for sensor values
 
  tcs.getRawData(&r, &g, &b, &c);                     // gets raw data from r g b c channels
  driveData.colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);     // converts raw data into single temp value, then sends it back to controller

    
  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
   if (commsLossCount > cMaxDroppedPackets) {
    failReboot();
  }

  int motorSpeed = map(inData.speed, 0, 4095, 0, 14);       // scale raw pot value into servo range 

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                     // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          // read and store current motor position
  }
  interrupts();                                       // turn interrupts back on

  uint32_t curTime = micros();                        // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle
    driveData.time = curTime;                         // update transmission time
    for (int k = 0; k < cNumMotors; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      // update target for set direction
      posChange[k] = (float) (inData.dir * motorSpeed); // update with maximum speed // use direction from controller
      targetF[k] = targetF[k] + posChange[k];         // set new target position
      if (k == 0) {                                   // assume differential drive
        target[k] = (int32_t) targetF[k];             // motor 1 spins one way
      }
      else {
        target[k] = (int32_t) -targetF[k];            // motor 2 spins in opposite direction
      }

      // use PID to calculate control signal to motor
      e[k] = target[k] - pos[k];                      // position error
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    // derivative of error
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    // integral of error (finite difference)
      u[k] = cKp * e[k] + cKd * dedt[k] + cKi * eIntegral[k]; // compute PID-based control signal
      ePrev[k] = e[k];                                // store error for next control cycle
  
      // set direction based on computed control signal
      dir[k] = 1;                                     // default to forward directon
      if (u[k] < 0) {                                 // if control signal is negative
        dir[k] = -1;                                  // set direction to reverse
      }

      // set speed based on computed control signal
      u[k] = fabs(u[k]);                              // get magnitude of control signal
      if (u[k] > cMaxSpeedInCounts) {                 // if control signal will saturate motor
        u[k] = cMaxSpeedInCounts;                     // impose upper limit
      }
      pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM); // convert control signal to pwm

      if (inData.left) {                              // if left button pushed, set power to 0
        pwm[0] = 0;
      }
      if (inData.right) {                             // if right button pushed, set power to 0
        pwm[1] = 0;
      }

      if (commsLossCount < cMaxDroppedPackets / 4) {
        setMotor(dir[k], pwm[k], cIN1Pin[k], cIN2Pin[k]); // update motor speed and direction
      }
      else {
        setMotor(0, 0, cIN1Pin[k], cIN2Pin[k]);       // stop motor
      }
#ifdef SERIAL_STUDIO
      if (k == 0) {
        printf("/*");                                 // start of sequence for Serial Studio parsing
      }
      printf("%d,%d,%d,%0.4f", target[k], pos[k], e[k], velMotor[k]);  // target, actual, error, velocity
      if (k < cNumMotors - 1) {
        printf(",");                                  // data separator for Serial Studio parsing
      }
      if (k == cNumMotors -1) {
        printf("*/\r\n");                             // end of sequence for Serial Studio parsing
      }
#endif
    }
    // send data from drive to controller
    if (peer->send_message((const uint8_t *) &driveData, sizeof(driveData))) {
      digitalWrite(cStatusLED, 0);                    // if successful, turn off communucation status LED
    }
    else {
      digitalWrite(cStatusLED, 1);                    // otherwise, turn on communication status LED
    }
  }
  doHeartbeat();                                      // update heartbeat LED
}

// blink heartbeat LED
void doHeartbeat() {
  uint32_t curMillis = millis();                      // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                     // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                              // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// function to reboot the device
void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
// triggered by rising edge of channel A
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);            // cast pointer to static structure
  
  int b = digitalRead(s->chanB);                      // read state of channel B
  if (b > 0) {                                        // B high indicates that it is leading channel A
    s->pos++;                                         // increase position
  }
  else {                                              // B low indicates that it is lagging channel A
    s->pos--;                                         // decrease position
  }
}