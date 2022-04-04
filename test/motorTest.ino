#include <sensors.h>

#define forwardPin 24
#define backPin 22
#define speedPin 2

Motor leftMotor(speedPin, forwardPin, backPin);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  leftMotor.forward(40);
  delay(1000);
  leftMotor.stop();
  delay(1000);
  leftMotor.backward(40);
  delay(1000);
  leftMotor.stop();
  delay(1000);
}