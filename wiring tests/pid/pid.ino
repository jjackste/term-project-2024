

// #define PRINT_SEND_STATUS                             
// #define PRINT_INCOMING                                

#include <Arduino.h>
#include "ESP32_NOW.h"

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
const int cIN1Pin[] = {17, 19, 23};                   // GPIO pin(s) for INT1
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
const float cKi = 0.1;                                // integral gain for PID
const float cKd = 0.1;                                // derivative gain for PID

// variables
uint32_t lastTime = 0;                                // last time of motor control was updated
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position (drive)
                     {32, 33, 0},                     // encoder 1 on GPIO 32 and 33, 0 position (drive)
                     {13, 27, 0}};                    // encoder 2 on GPIO 14 and 27, 0 position (collector)
int32_t target[] = {0, 0, 0};                         // target encoder count for motor
int32_t lastEncoder[] = {0, 0, 0};                    // encoder count at last control cycle
float targetF[] = {0.0, 0.0, 0.0};                    // target for motor as float

// added content
const int cNumMotors = 3;                        // number of DC motors including water wheel
const int sorterPin = 5;                         // pin for sorter servo
const int gatePin = 14;                          // pin for hopper gate servo
const int cTCSLED = 15;                          // GPIO pin for LED on TCS34725
const int cMinDutyCycle = 1700;                  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;                  // duty cycle for 180 degrees  
const int cStatusLED = 2;                        // status led
uint16_t r, g, b, c;                             // tcs values setup 
bool tcsFlag = 0;                                // tcs setup 
int lsTime = 0;                                  // timer count for sensor timer loop
int colourTemp = 0;                              // colourtemp set up
int spinDir = 0;                                 // spinDir setup
int pwmSaver = 1;
int runNumber = 1;
int count1 = 0;
int count2 = 0;


void setup() {
  Serial.begin(115200);                                      

  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);        
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);        
    pinMode(encoder[k].chanA, INPUT);                 
    pinMode(encoder[k].chanB, INPUT);                 
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
    }

  // initialize servo pins
  ledcAttach(sorterPin, 50, 16);                    // setup sorter pin for 50 Hz, 16-bit resolution
  ledcAttach(gatePin, 50, 16);                      // setup hopper gate pin for 50 Hz, 16-bit resolution

  // intialize status led
  pinMode(cStatusLED, OUTPUT);                      // configure GPIO for communication status LED as output

  // set collector motor to off
 
}

void loop() { 
  // motor variables
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

  noInterrupts();                                     
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          
    }
  interrupts();                                       

  int driveSpeed = analogRead(34);                       // drive pot value read
  int driveSpeed2 = map(driveSpeed, 0, 4095, 0, 14);

  // dc motor loop, both collection and drivetrain
  uint32_t curTime = micros();
  if (curTime - lastTime > 10000) {
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;
    lastTime = curTime;                             

    // collection motor driver, utilize variables from lab with a 3rd dc motor 
    velEncoder[2] = ((float) pos[2] - (float) lastEncoder[2]) / deltaT; 
    lastEncoder[2] = pos[2];                        
    velMotor[2] = velEncoder[2] / cCountsRev * 60;  

    posChange[2] = driveSpeed2;                                     // set with calculated rpm for optimal collection speed
    targetF[2] = targetF[2] + posChange[2];         
    target[2] = (int32_t) targetF[2];

    e[2] = target[2] - pos[2];
    Serial.println(e[2]);
    dedt[2] = ((float) e[2]- ePrev[2]) / deltaT;    
    eIntegral[2] = eIntegral[2] + e[2] * deltaT;    
    u[2] = cKp * e[2] + cKd * dedt[2] + cKi * eIntegral[2]; 
    ePrev[2] = e[2];                                

    u[2] = fabs(u[2]);                              
    if (u[2] > cMaxSpeedInCounts) {                 
      u[2] = cMaxSpeedInCounts;                     
    }
    pwm[2] = map(u[2], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM);  
    Serial.println(pwm[2]);

    setMotor(1, pwm[2], cIN1Pin[2], cIN2Pin[2]);  
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