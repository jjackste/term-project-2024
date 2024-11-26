int gatePin = 25;
int gatePot = 26;
const int cMinDutyCycle = 1700;  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;  // duty cycle for 180 degrees

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcAttach(gatePin, 50, 16);
}

void loop() {
  // put your main code here, to run repeatedly:
  int potVal = analogRead(gatePot);
  int gateVal = map(potVal, 0, 4095, 85, 180);
  int degree = degreesToDutyCycle(gateVal);
  ledcWrite(gatePin, degree);  // hopper gate control for depositing
  Serial.printf("analog value: %d, mapped value: %d, degree: %d\n", potVal, gateVal, degree);
}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
  return dutyCycle;
}