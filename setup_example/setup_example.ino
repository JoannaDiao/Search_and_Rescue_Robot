#define DEBUG_ON

#include <sensors.h>

// Digital pins connected to TOF sensors
#define SHT_LOX1 23
#define SHT_LOX2 25
#define SHT_LOX3 27

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

bool turn_test = true;
int left_motor_power;
int right_motor_power;


//ColorSensor colorSensor;


IMU imu;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(50, INPUT);
  pinMode(52, INPUT);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

//  Serial.println(F("Setting up TOF"));
//  pinMode(leftTOF.shutdownPin, OUTPUT);    
//  pinMode(frontTOF.shutdownPin, OUTPUT);
//  pinMode(backTOF.shutdownPin, OUTPUT);
//  delay(10);
//  
//  // all reset
//  digitalWrite(leftTOF.shutdownPin, LOW);    
//  digitalWrite(frontTOF.shutdownPin, LOW);
//  digitalWrite(backTOF.shutdownPin, LOW);
//  delay(10);
//
//  // all unreset
//  digitalWrite(leftTOF.shutdownPin, HIGH);    
//  digitalWrite(frontTOF.shutdownPin, HIGH);
//  digitalWrite(backTOF.shutdownPin, HIGH);
//  delay(10);
//
//  // activating leftTOF and resetting other two
//  digitalWrite(leftTOF.shutdownPin, HIGH);
//  digitalWrite(frontTOF.shutdownPin, LOW);
//  digitalWrite(backTOF.shutdownPin, LOW);
//  Serial.println(F("Here"));
//  leftTOF.init();
//  delay(10);
//  Serial.println(F("Set up TOF"));
//
//  digitalWrite(frontTOF.shutdownPin, HIGH);
//  frontTOF.init();
//  Serial.println(F("Set up front TOF"));
//
//  delay(10);
//  
//  digitalWrite(backTOF.shutdownPin, HIGH);
//  backTOF.init();
//  Serial.println(F("Set up all TOF"));

  // setup motors and encoders
  leftMotor.init();
  rightMotor.init();
  
  // then initialize imu
  imu.init();
  
  // then initialize color sensor

  //colorSensor.init();
}

double normalizeAngle(double angle) {
  angle = fmod(angle,360);
  if (angle < 0)
      angle += 360;
  return angle;
}

void loop() {
  imu.update();
  if (!turn_test) {
    if (digitalRead(50) == LOW && digitalRead(52) == LOW) {
      rightMotor.forward(60); 
      leftMotor.forward(60); 
    } else if (digitalRead(52) && digitalRead(50) == LOW) {
      rightMotor.backward(60); 
      leftMotor.backward(60); 
    } else {
      rightMotor.stop();
      leftMotor.stop();
    }
  } else {
    int initial_turn_heading = imu.getHeading();

    int target_heading = imu.getHeading() - 90 < 0 ? imu.getHeading() - 90 + 360 : imu.getHeading() - 90;
    while(abs(target_heading - imu.getHeading()) > 5) {
        imu.update();
        Serial.println(imu.getHeading());
        Serial.println(abs(target_heading - imu.getHeading()));
        left_motor_power = 40;
        right_motor_power = 40;
        leftMotor.backward(left_motor_power);
        rightMotor.forward(right_motor_power);
    }
    leftMotor.stop();
    rightMotor.stop();
    delay(5000);
  }
  
}
