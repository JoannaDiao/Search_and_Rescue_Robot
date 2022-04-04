#include <ArduinoSTL.h>

#define DEBUG_ON

#include <sensors.h>

// Digital pins connected to TOF sensors
#define SHT_LOX1 25
#define SHT_LOX2 27

// I2C addressing of TOF sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// motor pins
#define L_MOTOR_PWM 2
#define L_MOTOR_PIN1 22
#define L_MOTOR_PIN2 24
#define R_MOTOR_PWM 3
#define R_MOTOR_PIN1 26
#define R_MOTOR_PIN2 28


// Initialize a motor
Motor leftMotor(L_MOTOR_PWM, L_MOTOR_PIN1, L_MOTOR_PIN2);
Motor rightMotor(R_MOTOR_PWM, R_MOTOR_PIN1, R_MOTOR_PIN2);


// Initialize a TOF
TOF leftTOF(LOX1_ADDRESS, SHT_LOX1, true);
TOF frontTOF(LOX2_ADDRESS, SHT_LOX2, false);


//ColorSensor colorSensor;


//IMU imu;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println(F("Setting up TOF"));
  pinMode(leftTOF.shutdownPin, OUTPUT);    
  pinMode(frontTOF.shutdownPin, OUTPUT);
  delay(10);
  
  // all reset
  digitalWrite(leftTOF.shutdownPin, LOW);    
  digitalWrite(frontTOF.shutdownPin, LOW);
  delay(10);

  // all unreset
  digitalWrite(leftTOF.shutdownPin, HIGH);    
  digitalWrite(frontTOF.shutdownPin, HIGH);
  delay(10);

  // activating leftTOF and resetting other two
  digitalWrite(leftTOF.shutdownPin, HIGH);
  digitalWrite(frontTOF.shutdownPin, LOW);
  Serial.println(F("Setting up Left TOF..."));
  leftTOF.init();
  delay(10);
  Serial.print(F("...Done"));
  Serial.println(F(" "));
  digitalWrite(frontTOF.shutdownPin, HIGH);
  Serial.println(F("Setting up Front TOF..."));
  frontTOF.init();
  Serial.print(F("...Done"));
  Serial.println(F(" "));
  delay(10);
  
  Serial.println(F("Set up all TOF"));

  // setup motors and encoders
  leftMotor.init();
  rightMotor.init();
  
  // then initialize imu
  //imu.init();
  
  // then initialize color sensor

  //colorSensor.init();
}

void loop() {
  leftMotor.backward(80); 
  rightMotor.backward(80);

}
