#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ArduinoSTL.h>
//#include <HashMap.h>
#include <sensors.h>
//#include <microTuple.h>
//#include <map>

// --------------------------------- // PIN SETUP // ---------------------------------
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

// --------------------------------- // ENUMS // ---------------------------------
typedef enum {
  INIT,
  TILE_FORWARD,
  PIT_FORWARD,
  TURN_RIGHT,
  LEFT_ADJUST,
  RIGHT_ADJUST,
  STOP
} robot_state_t;

// --------------------------------- // CONSTANTS // ---------------------------------
// Motor Values
const int PIT_MOTOR_LOW = 40;
const int PIT_MOTOR_MED = 50;
const int PIT_MOTOR_HIGH = 60;
const int PIT_INCREMENT = 2; 
const int TILE_MOTOR_VALUE = 60;
const int TURN_MOTOR_VALUE_LEFT = 50;
const int TURN_MOTOR_VALUE_RIGHT = 50; 

// Course Dimensions
const double WIDTH = 1800;
const double TILE_WIDTH = 300;
const int MAX_TURNS = 10;

// Robot Dimensions
const int ROBOT_WIDTH = 150;
const int ROBOT_LENGTH = 190;

// Tolerance
const int ROBOT_MOTOR_OFFSET = 5;       //unused
const int RIGHT_ADJUST_DIST_TOL = -10;
const int RIGHT_ADJUST_ANGLE_TOL = 10;
const int LEFT_ADJUST_DIST_TOL = 10;
const int LEFT_ADJUST_ANGLE_TOL = 10;
const double CENTER_TILE_TOL = 10;      //unused
const double ADJUST_VALUE = 10;
const int TURN_TOLERANCE = 5;

// --------------------------------- // GLOBAL VARIABLES // ---------------------------------
// Robot States
//std::pair<double, double> robot_position;   //unused
//robot_orientation current_orientation;      //unused
robot_state_t robot_state = INIT;
bool justTurnt = false;

// Button Values
int button_state = 0;
const int button_pin = 2;

// Tile Values
int turnDistance = TILE_WIDTH - ROBOT_LENGTH;
int numTurns = 0;

// Motor Values
int left_motor_power = 0;
int right_motor_power = 0;

// Angle (IMU)
double prev_pitch = 0;

// TOF Values
int maxLeftTOFValue = 150;   // tile width - robot width
int minleftTOFValue = 100;   // tile width / 3

// offsets
int heading_offset = 0; 
int pitch_offset = 0; 
int left_tof_offset = 0;

// --------------------------------- // INITIALIZE HARDWARE // ---------------------------------
Motor left_motor(L_MOTOR_PWM, L_MOTOR_PIN1, L_MOTOR_PIN2);
Motor right_motor(R_MOTOR_PWM, R_MOTOR_PIN1, R_MOTOR_PIN2);

TOF front_tof(LOX2_ADDRESS, SHT_LOX2);
TOF back_tof(LOX3_ADDRESS, SHT_LOX3);
TOF left_tof(LOX1_ADDRESS, SHT_LOX1); 

IMU imu;

// --------------------------------- // FUNCTIONS // ---------------------------------
void setState(robot_state_t new_state) {
  robot_state = new_state;
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  delay(5000);
  imu.updateIMU();
  heading_offset = imu.getHeading(); 
  pitch_offset = imu.getPitch(); 
  left_tof_offset = left_tof.getDistance();
  
  setState(TILE_FORWARD);
}

// Determine which TOF to use and return the reading
int getPosition() {
    int front_tof_value = front_tof.getDistance(); 
    int back_tof_value = back_tof.getDistance();

    // Check if front TOF is valid
    char curr_TOF = front_tof_value == -1 ? 'B' : 'F';
    
    // return value depending on which TOF needs to be used
    return curr_TOF == 'B' ? back_tof_value : front_tof_value;
}

// Adjust the motor is specifed direction so that it is in the desired tolerance
void adjust(char direction) {
    if (direction == 'R') {
        left_motor_power = ADJUST_VALUE + TILE_MOTOR_VALUE;
    } else if (direction == 'L') {
        right_motor_power = ADJUST_VALUE + TILE_MOTOR_VALUE;
    }

    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);

    // When Adjustment is done, set robot to move forward
    if(abs(imu.getHeading() - heading_offset) < 10) {
      setState(TILE_FORWARD);
    }
}

/* FUNCTION: check if robot needs to adjust right or left, set state if needed
   CONDITIONS: 
        1. TOF reading not within desired tolerance
        2. actual TOF distance is too close/far to wall (out of range)
        3. IMU angle angle is too small/big
*/
bool needAdjust() {
    if ( (left_tof.getDistance() - left_tof_offset) < RIGHT_ADJUST_DIST_TOL 
            || left_tof.getDistance() < minleftTOFValue
            || (imu.getHeading() - heading_offset) > RIGHT_ADJUST_ANGLE_TOL ) {
        setState(RIGHT_ADJUST);
        return true;
    } else if ( (left_tof.getDistance() - left_tof_offset) > LEFT_ADJUST_DIST_TOL 
            || left_tof.getDistance() > maxLeftTOFValue
            || (imu.getHeading() - heading_offset) < LEFT_ADJUST_ANGLE_TOL ) {
        setState(LEFT_ADJUST);
        return true;
    }

    return false;
}

// Moves the robot forward, calls needAdjust()
void moveForward () {
    // Set motor power to forward
    left_motor_power = TILE_MOTOR_VALUE;
    right_motor_power = TILE_MOTOR_VALUE;

    // Allow robot to try and turn again
    if(justTurnt && getPosition() > turnDistance) {
        justTurnt = false;
    }

    // check if robot needs to adjust right or left, set state if needed and break
    if (needAdjust()) {
        return;
    }

    // Turn motors on
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power); 

}

// Stops robot and turns right 90 degrees. Check if distance is too far, call adjust.
void turnRight() {
    stop();

    // Determine desired heading robot (current position +/- 90 degrees)
    int target_heading =  heading_offset - 90 < 0 ? heading_offset + 270 : heading_offset - 90; 

    // While motor has not turned 90 degrees, continue to turn clockwise
    while (abs(imu.getHeading() - target_heading) > TURN_TOLERANCE) {
        imu.updateIMU();
        left_motor_power = TURN_MOTOR_VALUE_LEFT;
        right_motor_power = TURN_MOTOR_VALUE_RIGHT;
        left_motor.backward(TURN_MOTOR_VALUE_LEFT);
        right_motor.forward(right_motor_power);
    }

    // Adjust robot until it is in range (100 - 150 from wall)
    while(left_tof.getDistance() > maxLeftTOFValue) {
        adjust('R');
    }

    while (left_tof.getDistance() < minleftTOFValue) {
        adjust('L');
    }

    // get new correct distance to wall (the distance to keep straight at)
    left_tof_offset = left_tof.getDistance(); 

    // calculate new correct heading
    heading_offset =  heading_offset - 90 < 0 ? heading_offset + 270 : heading_offset - 90; 

    // Update system parameters
    numTurns = numTurns + 1;
    updateRequiredDistance();
    justTurnt = true;
}

void updateRequiredDistance () {
     // Change min turn distance for front TOF, every 4 turns 
    if(numTurns % 3 == 0) {
        turnDistance = turnDistance + TILE_WIDTH;
    }

    // Change max distance expected from left TOF, every 4 turns
    if (numTurns % 4 == 0) {
        maxLeftTOFValue = maxLeftTOFValue + TILE_WIDTH;
        minleftTOFValue = minleftTOFValue + TILE_WIDTH;
    }
}

// Stops the motor
void stop() {
    left_motor.stop();
    right_motor.stop();
}

// --------------------------------- // SETUP AND LOOP // ---------------------------------
void setup() {
    Serial.begin(115200);

    // wait until serial port opens for native USB devices
    while (!Serial) {
        delay(1);
    }

    Serial.println(F("Setting up TOF")); 
    pinMode(left_tof.shutdownPin, OUTPUT);
    pinMode(front_tof.shutdownPin, OUTPUT);
    pinMode(back_tof.shutdownPin, OUTPUT);
    delay(10);

    // all reset   
    digitalWrite(left_tof.shutdownPin, LOW);
    digitalWrite(front_tof.shutdownPin, LOW);
    digitalWrite(back_tof.shutdownPin, LOW);
    delay(10);

    // all unreset   
    digitalWrite(left_tof.shutdownPin, HIGH);
    digitalWrite(front_tof.shutdownPin, HIGH);
    digitalWrite(back_tof.shutdownPin, HIGH);
    delay(10);

    // activating leftTOF and resetting other two
    digitalWrite(left_tof.shutdownPin, HIGH);
    digitalWrite(front_tof.shutdownPin, LOW);
    digitalWrite(back_tof.shutdownPin, LOW);
    left_tof.init();

    delay(10);
    Serial.println(F("Set up left TOF"));

    digitalWrite(front_tof.shutdownPin, HIGH);
    front_tof.init();
    Serial.println(F("Set up front TOF"));

    delay(10);

    digitalWrite(back_tof.shutdownPin, HIGH);
    back_tof.init();
    Serial.println(F("Set up all TOF"));

    // setup motors and encoders
    left_motor.init();
    right_motor.init();

    // then initialize imu
    imu.init();

    pinMode(button_pin, INPUT);
    setState(INIT);

    //Unused
    //robot_position.first = 0; 
    //robot_position.second = 0;

    Serial.println(F("Set up all sensors"));
}

void loop() {
    imu.updateIMU();

    int curr_position = getPosition();

    // default to always move straight
    setState(TILE_FORWARD);

    if (curr_position <= turnDistance && !justTurnt) {
        setState(TURN_RIGHT);
    } else if (numTurns == MAX_TURNS) {
        setState(STOP);
    }

    switch (robot_state) {
    case INIT:
        handleInit(); 
        break;
    case TILE_FORWARD:
        moveForward();
        break;
    case TURN_RIGHT:
        turnRight();
        break;
    case LEFT_ADJUST:
        adjust('L');
        break;
    case RIGHT_ADJUST:
        adjust('R');
        break;
    case STOP:
        stop();
        break;
  }

  delay(200); 
}
