//
// term project 2024 robot code  
// lab 004, team 1
// 

//
// controllable functions:
// go forward and reverse
// turn left and right while moving
// tank turn left and right stationary
// open / close hopper gate
// 
// automated functions:
// colour sensing
// colour sorting
// collector wheel 
// reboot in case of failure
//

// #define PRINT_SEND_STATUS                             
// #define PRINT_INCOMING                                

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X); 

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 
#define ESPNOW_WIFI_CHANNEL 4                         

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

// encoder structure
struct Encoder {
  const int chanA;                                    
  const int chanB;                                    
  int32_t pos;                                        
};

// function declarations
void setMotor(int dir, int pwm, int in1, int in2);
void failReboot();
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// motor constants
const int cNumDriveMotors = 2;                        // number of drive motors
const int cIN1Pin[] = {17, 19, 15};                   // GPIO pin(s) for INT1
const int cIN2Pin[] = {16, 18, 4};                    // GPIO pin(s) for INT2
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

// variables
uint32_t lastTime = 0;                                // last time of motor control was updated
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position (drive)
                     {32, 33, 0},                     // encoder 1 on GPIO 32 and 33, 0 position (drive)
                     {14, 27, 0}};                    // encoder 2 on GPIO 14 and 27, 0 position (collector)
int32_t target[] = {0, 0, 0};                         // target encoder count for motor
int32_t lastEncoder[] = {0, 0, 0};                    // encoder count at last control cycle
float targetF[] = {0.0, 0.0, 0.0};                    // target for motor as float

// communication
uint8_t receiverMacAddress[] = {0xA8,0x42,0xE3,0xCA,0xF1,0xBC};  // MAC address of controller 00:01:02:03:04:05
esp_now_control_data_t inData;                        // control data packet from controller
esp_now_drive_data_t driveData;                       // data packet to send to controller

// added content
const int cNumMotors = 3;                        // number of DC motors including water wheel
const int sorterPin = 5;                         // pin for sorter servo
const int gatePin = 13;                          // pin for hopper gate servo
const int cTCSLED = 23;                          // GPIO pin for LED on TCS34725
const int cMinDutyCycle = 1400;                  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;                  // duty cycle for 180 degrees  
const int cStatusLED = 2;                        // status led
uint16_t r, g, b, c;                             // tcs values setup 
bool tcsFlag = 0;                                // tcs setup 
int lsTime = 0;                                  // timer count for sensor timer loop
int colourTemp = 0;                              // colourtemp set up
int spinDir = 0;                                 // spinDir setup

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
    if (len == 0)                                       // if empty packet
    {
      return;                                           // return
    }
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller
    #ifdef PRINT_INCOMING
      Serial.printf("f/r: %d, drveSpeed: %d, left: %d, right: %d,  gate: %d, time: %d\n", inData.dir, inData.driveSpeed, inData.left, inData.right, inData.hopper, inData.time);
    #endif
    }
  
  
  void onSent(bool success) {
    if (success) {
    #ifdef PRINT_SEND_STATUS
        log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
        Serial.printf("water pwm: %d, colour temp: %d, result: %d\n", driveData.waterSpeed, driveData.colourTemp, driveData.spinDir);
    #endif
      commsLossCount = 0;
    }
    else {
      commsLossCount++;
    }
    }
  
};

ESP_NOW_Network_Peer *peer;

void setup() {
  Serial.begin(115200);                               
  while (!Serial) {                                   
    delay(10);                                        
    }
  WiFi.mode(WIFI_STA);                                
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               
  while (!WiFi.STA.started()) {                       
    delay(100);                                       
   }
  Serial.print("MAC address for drive "); 
  Serial.println(WiFi.macAddress());                  

  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
   }
  
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
  memset(&inData, 0, sizeof(inData));                 
  memset(&driveData, 0, sizeof(driveData));           

  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);        
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);        
    pinMode(encoder[k].chanA, INPUT);                 
    pinMode(encoder[k].chanB, INPUT);                 
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
    }
  ledcAttach(sorterPin, 50, 16);                    // setup sorter pin for 50 Hz, 16-bit resolution
  ledcAttach(gatePin, 50, 16);                      // setup hopper gate pin for 50 Hz, 16-bit resolution
  pinMode(cStatusLED, OUTPUT);                      // configure GPIO for communication status LED as output

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
  float deltaT = 0;                                      
  int32_t pos[] = {0, 0, 0};                             
  int32_t e[] = {0, 0, 0};                               
  float velEncoder[] = {0, 0, 0};                        
  float velMotor[] = {0, 0, 0};                          
  float posChange[] = {0, 0, 0};                         
  float ePrev[] = {0, 0, 0};                             
  float dedt[] = {0, 0, 0};                              
  float eIntegral[] = {0, 0, 0};                          
  float u[] = {0, 0, 0};                                 
  int pwm[] = {0, 0, 0};                                 
  int dir[] = {1, 1, 1};                                 

  if (commsLossCount > cMaxDroppedPackets) {
      failReboot();
  }

  noInterrupts();                                     
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          
    }
  interrupts();                                       

  // hopper gate control
  ledcWrite(gatePin, degreesToDutyCycle(inData.hopper)); // hopper gate control for depositing, 

  // sorter time loop, sense and sort 
  uint32_t cTime = millis();                       // capture current time in milliseconds
  if (cTime - lsTime > 1000) {                     // wait ~1 s
    lsTime = cTime;
    tcs.getRawData(&r, &g, &b, &c);                                                         // get raw RGBC values
    colourTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);                            // convert to arbritary constant for comparision
    Serial.printf("colour temp: %d, r: %d, g: %d, b: %d, c: %d\n", colourTemp, r, g, b, c); // troubeshooting help

    // sorter servo control
    if ( (colourTemp >= 3800) && (colourTemp <= 4500)  && (r <= 30) && (g <= 30) && (b <= 20) ) { // baseline values for servo to stay in middle, either after scanning slide face or open air
      ledcWrite(sorterPin, degreesToDutyCycle(90)); // spin middle
      // Serial.printf("baseline \n");
      spinDir = 0;
      } 
      else if ( (colourTemp >= 4000) && (colourTemp <= 5000) && (c >= 80) && (r <= 80)) { // good or wanted values, spin left
      ledcWrite(sorterPin, degreesToDutyCycle(20)); // spin left
      // Serial.printf("good \n");
      spinDir = 1;
      }
      else if ( colourTemp = 0) { // restart esp32 if colour sensor stops working, unknown cause
        failReboot();
      } 
      else { // any other value is bad, spin to the back
      ledcWrite(sorterPin, degreesToDutyCycle(160)); // spin right
      // Serial.printf("bad \n");
      spinDir = 2;
    }
    driveData.spinDir = spinDir;
  }

  // dc motor loop
  uint32_t curTime = micros();
  if (curTime - lastTime > 10000) {
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;
    lastTime = curTime;
    driveData.time = curTime;                                  // update transmission time

    // water wheel motor driver
    velEncoder[2] = ((float) pos[2] - (float) lastEncoder[2]) / deltaT; 
    lastEncoder[2] = pos[2];                        
    velMotor[2] = velEncoder[2] / cCountsRev * 60;  

    posChange[2] = 10;                                         // set with calculated speed for optimal collection speed
    targetF[2] = targetF[2] + posChange[2];         
    target[2] = (int32_t) targetF[2];

    e[2] = target[2] - pos[2];
    dedt[2] = ((float) e[2]- ePrev[2]) / deltaT;    
    eIntegral[2] = eIntegral[2] + e[2] * deltaT;    
    u[2] = cKp * e[2] + cKd * dedt[2] + cKi * eIntegral[2]; 
    ePrev[2] = e[2];                                

    u[2] = fabs(u[2]);                              
    if (u[2] > cMaxSpeedInCounts) {                 
      u[2] = cMaxSpeedInCounts;                     
    }
    pwm[2] = map(u[2], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM);  
    driveData.waterSpeed = pwm[2];                              // send calculated pwm back to controller

    if (commsLossCount < cMaxDroppedPackets / 4) {
      setMotor(-1, pwm[2], cIN1Pin[2], cIN2Pin[2]); 
      // Serial.printf("water motor pwm: %d \n", pwm[2]);
      }
    else {
      setMotor(0, 0, cIN1Pin[2], cIN2Pin[2]);       
    }

    // drivetrain motors
    for (int k = 0; k <= 1; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; 
      lastEncoder[k] = pos[k];                        
      velMotor[k] = velEncoder[k] / cCountsRev * 60;

      if (inData.left && inData.dir == 0) {           // if case switcher to see if only left or right button pressed w/o any froward or revers
          posChange[0] = inData.driveSpeed;           // over ride the inData.dir * motorSpeed to force the same direction of the motors
          posChange[1] = -inData.driveSpeed;          // because lower if k == 0 target = +/- targetF case, the directions have to be flopped
      } else if (inData.right && inData.dir == 0) {
          posChange[0] = -inData.driveSpeed;
          posChange[1] = inData.driveSpeed;
      } else {
        posChange[k] = (float) (inData.dir * inData.driveSpeed); // update with maximum speed // use direction coming in from controller
      }
      targetF[k] = targetF[k] + posChange[k];         // set new target position

      if (k == 0) {                                   // assume differential drive
        target[k] = (int32_t) targetF[k];             // motor 1 spins one way
      }
      else {
        target[k] = (int32_t) targetF[k];             // motor 2 spins in opposite direction
      }

      e[k] = target[k] - pos[k];                      
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    
      u[k] = cKp * e[k] + cKd * dedt[k] + cKi * eIntegral[k]; 
      ePrev[k] = e[k];                                
  
      dir[k] = 1;                                     
      if (u[k] < 0) {                                 
        dir[k] = -1;                                 
      }

      u[k] = fabs(u[k]);                              
      if (u[k] > cMaxSpeedInCounts) {
        u[k] = cMaxSpeedInCounts;                     
      }

      pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM); 

      if (inData.left && inData.dir != 0) {           // if left and either front or back, kill power to one side    
        pwm[0] = 0;
      } else if (inData.right && inData.dir != 0) {   // if right and either front or back, kill power to other side
        pwm[1] = 0;
      }    

      if (commsLossCount < cMaxDroppedPackets / 4) {
        setMotor(dir[k], pwm[k], cIN1Pin[k], cIN2Pin[k]);
        Serial.printf("left speed: , right speed")
      }
      else {
        setMotor(0, 0, cIN1Pin[k], cIN2Pin[k]);       
      }
    }
  }
  if (peer->send_message((const uint8_t *) &driveData, sizeof(driveData))) {
        digitalWrite(cStatusLED, 1);
      }
      else {
        digitalWrite(cStatusLED, 0);
      }
}

void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                              
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);
  
  int b = digitalRead(s->chanB);
  if (b > 0) {                                        
    s->pos++;                                         
  }
  else {                                              
    s->pos--;                                         
  }
}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
  return dutyCycle;
}