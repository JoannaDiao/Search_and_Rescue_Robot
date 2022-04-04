#define DEBUG_ON

#include <sensors.h>

// Digital pins connected to TOF sensors
#define SHT_LOX1 25
#define SHT_LOX2 27
#define SHT_LOX3 29

// I2C addressing of TOF sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

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

// Initialize an encoder
// Encoder leftEncoder(30, 32);
// Encoder rightEncoder(31, 33);

// Initialize a TOF
TOF leftTOF(LOX1_ADDRESS, SHT_LOX1);
TOF frontTOF(LOX2_ADDRESS, SHT_LOX2);
TOF backTOF(LOX3_ADDRESS, SHT_LOX3);


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
  pinMode(backTOF.shutdownPin, OUTPUT);
  delay(10);
  
  // all reset
  digitalWrite(leftTOF.shutdownPin, LOW);    
  digitalWrite(frontTOF.shutdownPin, LOW);
  digitalWrite(backTOF.shutdownPin, LOW);
  delay(10);

  // all unreset
  digitalWrite(leftTOF.shutdownPin, HIGH);    
  digitalWrite(frontTOF.shutdownPin, HIGH);
  digitalWrite(backTOF.shutdownPin, HIGH);
  delay(10);

  // activating leftTOF and resetting other two
  digitalWrite(leftTOF.shutdownPin, HIGH);
  digitalWrite(frontTOF.shutdownPin, LOW);
  digitalWrite(backTOF.shutdownPin, LOW);
  Serial.println(F("Here"));
  leftTOF.init();
  delay(10);
  Serial.println(F("Set up TOF"));

  digitalWrite(frontTOF.shutdownPin, HIGH);
  frontTOF.init();
  delay(10);
  
  digitalWrite(backTOF.shutdownPin, HIGH);
  backTOF.init();
  Serial.println(F("Set up all TOF"));

  // setup motors and encoders
  leftMotor.init();
  rightMotor.init();
  
  // then initialize imu
  imu.init();
  
  // then initialize color sensor

  //colorSensor.init();
}

void loop() {
  

}