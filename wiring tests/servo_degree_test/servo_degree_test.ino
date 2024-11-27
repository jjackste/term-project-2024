int gatePin = 5;
const int cMinDutyCycle = 1700;  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;  // duty cycle for 180 degrees

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcAttach(gatePin, 50, 16);
}

void loop() {
  // put your main code here, to run repeatedly:
  ledcWrite(gatePin, degreesToDutyCycle(0));  // hopper gate control for depositing
  delay(500);
  ledcWrite(gatePin, degreesToDutyCycle(180)); 
  delay(500);
}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
  return dutyCycle;
}