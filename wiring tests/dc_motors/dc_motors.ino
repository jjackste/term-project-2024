struct Encoder {
  const int chanA;                                    // GPIO pin for encoder channel A
  const int chanB;                                    // GPIO pin for encoder channel B
  int32_t pos;                                        // current encoder position
};

void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

const int cNumDriveMotors = 2;                        // Number of DC motors
const int cNumMotors = 3;                        // Number of DC motors
const int cIN1Pin[] = {17, 19, 22};                       // GPIO pin(s) for INT1
const int cIN2Pin[] = {16, 18, 23};                       // GPIO pin(s) for INT2
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

uint32_t lastTime = 0;                                // last time of motor control was updated
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position
                     {32, 33, 0},                     // encoder 1 on GPIO 32 and 33, 0 position
                     {14, 27, 0}};                    // encoder 2 on GPIO 12 and 13, 0 position
int32_t target[] = {0, 0, 0};                         // target encoder count for motor
int32_t lastEncoder[] = {0, 0, 0};                    // encoder count at last control cycle
float targetF[] = {0.0, 0.0, 0.0};                    // target for motor as float

void setup() {
  Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);        // setup INT1 GPIO PWM channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);        // setup INT2 GPIO PWM channel
    pinMode(encoder[k].chanA, INPUT);                 // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                 // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }
}

void loop() {
  float deltaT = 0;                                      // time interval
  int32_t pos[] = {0, 0, 0};                             // current motor positions
  int32_t e[] = {0, 0, 0};                               // position error
  float velEncoder[] = {0, 0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0, 0};                         // change in position for set speed
  float ePrev[] = {0, 0, 0};                             // previous position error
  float dedt[] = {0, 0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0, 0};                         // integral of error 
  float u[] = {0, 0, 0};                                 // PID control signal
  int pwm[] = {0, 0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1, 1};                                 // direction that motor should turn

    // dc motor loop
  uint32_t curTime = micros();                        // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle

    // water wheel
    setMotor(1, 220, cIN1Pin[2], cIN2Pin[2]); // update motor speed and direction

    // drivetrain motors
    for (int k = 0; k <= 1; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      setMotor(1, 220, cIN1Pin[k], cIN2Pin[k]); // update motor speed and direction
    }
  }
}

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

// encoder interrupt service routine
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

