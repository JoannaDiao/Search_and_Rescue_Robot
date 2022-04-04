
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ArduinoSTL.h>
#include <HashMap.h>
#include <sensors.h>
#include <microTuple.h>
#include <map>
#include <ArduPID.h>


// Digital pins connected to TOF sensors
#define SHT_LOX1 30
#define SHT_LOX2 32

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

typedef enum {
  INIT,
  TILE_FORWARD,
  PIT_FORWARD,
  TURN_RIGHT,
  LEFT_ADJUST,
  RIGHT_ADJUST,
  STOP
} robot_state_t;

typedef enum {
  LEFT,
  TOP,
  RIGHT,
  BOTTOM
} robot_orientation;

robot_state_t robot_state = INIT;
int button_state = 0;
const int button_pin = 2;
int current_tile = 0;
robot_orientation current_orientation;
std::pair<double, double> robot_position;
const double WIDTH = 1800;
const double TILE_WIDTH = 300;
int left_motor_power = 0;
int right_motor_power = 0;

// PID Controllers
ArduPID adjustController;
double adjust_setpoint = 100;
double adjust_input;
double adjust_output;
double adjust_p = 605; // 25
double adjust_i = 0; //3
double adjust_d = 50000; //5

const int PIT_MOTOR_LOW = 40;
const int PIT_MOTOR_MED = 50;
const int PIT_MOTOR_HIGH = 95;
const int PIT_INCREMENT = 2; 
const int TILE_MOTOR_VALUE = 70;
const int TURN_MOTOR_VALUE = 50;
const int SLOW_MOTOR_VALUE = 40;
const int TILE_ADJUST_VALUE = 60;

const int ROBOT_WIDTH = 150;
const int ROBOT_LENGTH = 190;
const int LEFT_DIST_TOL = 30;
const int LEFT_DIST_ADJUST_TOL = 20;

const double CENTER_TILE_TOL = 10;

const double PITCH_UPWARDS_VALUE = -30;
const double PITCH_DOWNWARDS_VALUE = 60;
const double PITCH_TOL_VALUE = 50;
const double ADJUST_VALUE = 25;

const int RIGHT_ADJUST_DIST_TOL = 10;
const int RIGHT_ADJUST_ANGLE_TOL = 10;
const int LEFT_ADJUST_DIST_TOL = 10;
const int LEFT_ADJUST_ANGLE_TOL = -10;

// Initialize a motor
Motor left_motor(L_MOTOR_PWM, L_MOTOR_PIN1, L_MOTOR_PIN2);
Motor right_motor(R_MOTOR_PWM, R_MOTOR_PIN1, R_MOTOR_PIN2);

TOF front_tof(LOX2_ADDRESS, SHT_LOX2, false);
TOF left_tof(LOX1_ADDRESS, SHT_LOX1, true); 

IMU imu;

int curr_turn = 0;
float heading_offset = 0; 
float pitch_offset = 0; 
int des_left_offset = 0;

int last_tof_time = 0;
int prev_tof_value = 0;
float roc = 0;

// S = start, E = end, T = turn
std::map<std::pair<int, int>, char> course = {
  {{5, 4}, 'S'}, {{5, 0}, 'T'}, {{0, 0}, 'T'}, {{0, 5}, 'T'},
  {{4, 5}, 'T'}, {{4, 1}, 'T'}, {{1, 1}, 'T'}, {{1, 4}, 'T'},
  {{3, 4}, 'T'}, {{3, 2}, 'T'}, {{2, 2}, 'T'}, {{2, 3}, 'E'}
};

std::map<std::pair<int, int>, robot_orientation> turn_orientation = {
  {{5, 0}, BOTTOM}, {{0, 0}, LEFT}, {{0, 5}, TOP},
  {{4, 5}, RIGHT}, {{4, 1}, BOTTOM}, {{1, 1}, LEFT}, {{1, 4}, TOP},
  {{3, 4}, RIGHT}, {{3, 2}, BOTTOM}, {{2, 2}, TOP}
};

std::vector<std::pair<int,int>> turn_order = {
  {5, 0}, {0, 0}, {0, 5},
  {4, 5}, {4, 1}, {1, 1}, {1, 4},
  {3, 4}, {3, 2}, {2, 2}
};


// F = front tof, B = back tof, x and y in mm, based on 1.8 m
std::vector<std::vector<std::pair<int, int>>> coords = {
  {{150, 150}, {450, 150}, {750, 150}, {1050, 150}, {1350, 150}, {1650, 150}},
  {{150, 450}, {450, 480}, {750, 450}, {1050, 450}, {1320, 450}, {1650, 450}},
  {{150, 750}, {450, 750}, {730, 750}, {1050, 730}, {1350, 750}, {1650, 750}},
  {{150, 1050}, {450, 1050}, {760, 1080}, {1050, 1050}, {1360, 1000}, {1650, 1050}},
  {{150, 1350}, {480, 1360}, {750, 1350}, {1050, 1350}, {1350, 1350}, {1680, 1320}},
  {{150, 1650}, {450, 1650}, {750, 1650}, {1050, 1650}, {1350, 1650}, {1650, 1650}}
};

std::vector<std::pair<int, int>> path = {
  {5,4}, {5, 3}, {5, 2}, {5, 1}, {5, 0}, {4, 0}, {3, 0},
  {2, 0}, {1, 0}, {0, 0}, {0, 1}, {0, 2}, {0, 3},
  {0, 4}, {0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5},
  {4, 4}, {4, 3}, {4, 2}, {4, 1}, {3, 1}, {2, 1},
  {1, 1}, {1, 2}, {1, 3}, {1, 4}, {2, 4}, {3, 4},
  {3, 3}, {3, 2}, {2, 2}, {2, 3}
};

/* ------------ Functions --------------- */

void setState(robot_state_t new_state) {
  robot_state = new_state;
//  Serial.println(robot_state);
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  int left_tof_value = left_tof.getDistance();
  int front_tof_value = front_tof.getDistance();
  
  Serial.println(" "); 
  Serial.print("TOF VALUES "); 
  Serial.print(left_tof_value); 
  Serial.print("   "); 
  Serial.print(front_tof_value); 
  Serial.println(" "); 

  if (left_tof_value == -1 || front_tof_value == -1) {
    return;   
  }
  
  switch (current_orientation) {
    case LEFT:
      position.first = left_tof_value + ROBOT_WIDTH / 2.0;
      position.second = front_tof_value + ROBOT_LENGTH / 2.0;
      break;
    case RIGHT:
      position.first = WIDTH - left_tof_value - ROBOT_WIDTH / 2.0;
      position.second = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      break;
    case TOP:
      position.first = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      position.second = left_tof_value + ROBOT_WIDTH / 2.0;
      break;
    case BOTTOM:
      position.first = front_tof_value + ROBOT_LENGTH / 2.0; 
      position.second = WIDTH - left_tof_value - ROBOT_WIDTH / 2.0;
      break;
  }  
}

float getRateOfChange(int first, int last) {
  float dt = (micros() - last_tof_time) * 1e-6;
  last_tof_time = micros();

  return (last - first) / dt;  
}
void updateCurrentTile(const std::pair<double, double>& position, const int next_tile, int& current_tile) {
/*
  if(left_tof.getDistance() == -1 || front_tof.getDistance() == -1) {
    return;
  } */
  
  std::pair<int, int> coord = coords[path[next_tile].first][path[next_tile].second];
  double next_center_x = coord.first;
  double next_center_y = coord.second;
  double position_x = position.first;
  double position_y = position.second;

  // check if we're in the next tile
  if (position_x <= next_center_x + TILE_WIDTH / 2.0 && position_x > next_center_x - TILE_WIDTH / 2.0
      && position_y <= next_center_y + TILE_WIDTH / 2.0 && position_y > next_center_y - TILE_WIDTH / 2.0) {
    current_tile = next_tile;
    return;
  } 


/*
  int front_tof_value = front_tof.getDistance();
  if(left_tof.getDistance() == -1 || front_tof_value == -1) {
    return;
  } */ 
  
  // check if we're not in the current tile and if we're not, where the hell are we?
  std::pair<int, int> curr_coord = coords[path[current_tile].first][path[current_tile].second];
  double curr_center_x = curr_coord.first;
  double curr_center_y = curr_coord.second;
  double curr_x = position.first;
  double curr_y = position.second;
  /*
  switch (current_orientation) {
    case LEFT:
      curr_x = des_left_offset + ROBOT_WIDTH / 2.0;
      curr_y = front_tof_value + ROBOT_LENGTH / 2.0;
      break;
    case RIGHT:
      curr_x = WIDTH - des_left_offset - ROBOT_WIDTH / 2.0;
      curr_y = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      break;
    case TOP:
      curr_x = WIDTH - front_tof_value - ROBOT_LENGTH / 2.0; 
      curr_y = des_left_offset + ROBOT_WIDTH / 2.0;
      break;
    case BOTTOM:
      curr_x = front_tof_value + ROBOT_LENGTH / 2.0; 
      curr_y = WIDTH - des_left_offset - ROBOT_WIDTH / 2.0;
      break;
  } */
  
  if (curr_x > curr_center_x + TILE_WIDTH / 2.0 || curr_x < curr_center_x - TILE_WIDTH / 2.0
      || curr_y > curr_center_y + TILE_WIDTH / 2.0 || curr_y < curr_center_y - TILE_WIDTH / 2.0) {
        Serial.println("FIND WHERE WE ARE..");
    
    // iterate through coords and find the tile we're on 
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            std::pair<int, int> ij_coord = coords[i][j];

            double ij_center_x = ij_coord.first;
            double ij_center_y = ij_coord.second;
            
            if (curr_x <= ij_center_x + TILE_WIDTH / 2.0 && curr_x > ij_center_x - TILE_WIDTH / 2.0
                && curr_y <= ij_center_y + TILE_WIDTH / 2.0 && curr_y > ij_center_y - TILE_WIDTH / 2.0) {
                // iterate through path to get current_tile
                for(int k = 0; k < path.size(); k++) {
                  if(path[k].first == i && path[k].second == j) {
                      current_tile = k; 
                      Serial.print("..FOUND where we are..");
                      Serial.print(current_tile);
                      Serial.println(" ");
                      return;
                  }
                }
             }
          }
      }
  } 
}

// returns desired left tof value 
int getDesLeftDist() {
  std::pair<int, int> coord = coords[path[current_tile].first][path[current_tile].second];
  
  switch (current_orientation) {
    case LEFT:
      return coord.first - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case RIGHT:
      return WIDTH - coord.first - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case TOP:
      return coord.second - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
    case BOTTOM:
      return WIDTH - coord.second - ROBOT_WIDTH / 2 + LEFT_DIST_TOL;
  }
  return 0;
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  Serial.println("HandleInit");
  
  int start = micros();
  while(micros() - start < 5e6) {
    imu.updateIMU();
  }

  heading_offset = imu.getHeading(); 
 // pitch_offset = imu.getGyroPitch(); 
  des_left_offset = getDesLeftDist();
  adjust_setpoint = des_left_offset;
  last_tof_time = micros();
  
  prev_tof_value = front_tof.getDistance();
  
  setState(TILE_FORWARD);
}

void handleTileForward() {
  Serial.println("HandleTileForward");

  /*
  if (imu.getGyroPitch() > PITCH_DOWNWARDS_VALUE) {
    setState(PIT_FORWARD);
    return;
  } */

  std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
  if (it != course.end()) {
    char landmark = it->second;

    if (landmark == 'T' && curr_turn < 11) {
      std::map<std::pair<int, int>, robot_orientation>::iterator it_turn = turn_orientation.find(path[current_tile]);
      if(it_turn != turn_orientation.end() && it_turn->second == current_orientation && path[current_tile] == turn_order[curr_turn]) {
          if(abs(imu.getGyroPitch()) > PITCH_TOL_VALUE || robot_state == PIT_FORWARD || abs(imu.getRoll()) > PITCH_TOL_VALUE) {
            delay(500);
            return;
          } 
          setState(TURN_RIGHT);
          return;
      }
    } else if (landmark == 'E' && curr_turn > 10) {
      setState(STOP);
      return;
    }
  }

  float PID_output = adjust_output / 255.0;
  left_motor_power = TILE_MOTOR_VALUE + 20*PID_output;
  right_motor_power = TILE_MOTOR_VALUE - 20*PID_output;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 
}

void handlePitForward() {
  Serial.println("handlePitForward");
  Serial.println(" ");
  Serial.print("Pitch: ");
  Serial.print(imu.getGyroPitch());
  Serial.println(" ");
  delay(500);
  left_motor_power = PIT_MOTOR_HIGH;
  right_motor_power = PIT_MOTOR_HIGH;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power);
  
  if(imu.getGyroPitch() < PITCH_UPWARDS_VALUE) {
    
    left_motor_power = PIT_MOTOR_HIGH;
    right_motor_power = PIT_MOTOR_HIGH;
    
    left_motor.backward(left_motor_power);
    right_motor.backward(right_motor_power);
      
    // exit pit state
    /*
    while(abs(imu.getGyroPitch()) > 15) {
      imu.updateIMU();   
      left_motor_power = PIT_MOTOR_HIGH;
      right_motor_power = PIT_MOTOR_HIGH;
  
      left_motor.backward(left_motor_power);
      right_motor.backward(right_motor_power);

      Serial.println("WE ARE STUCK");
    }
      */

    setState(TILE_FORWARD);
  } 
}

void handleTurnRight() {
  Serial.println("handleTurnRight"); 
  adjustController.stop();
  // is our front_tof value in the middle of the tile ? if not return; 
  
  std::pair<int, int> coord = coords[path[current_tile].first][path[current_tile].second];
  std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
  if (it != course.end()) {
    char landmark = it->second;

    if (landmark != 'T') {
      setState(TILE_FORWARD);
      return;
    }
  } 

  double front_dist_tolerance = 400;
    switch (current_orientation) {
    case LEFT:
      if(robot_position.second > coord.second + front_dist_tolerance) return;   
    case RIGHT:
      if(robot_position.second < coord.second - front_dist_tolerance) return;
    case TOP:
      if(robot_position.first < coord.first - front_dist_tolerance) return;
    case BOTTOM:
      if(robot_position.first > coord.first + front_dist_tolerance) return;
  } 
  
  // turn !! 
  Serial.println("WE ARE GOING TO STOP");
  left_motor.stop();
  right_motor.stop(); 
  
  double curr_heading = imu.getHeading();
  double target_heading = curr_heading - 90 < 0 ? curr_heading - 90 + 360 : curr_heading - 90; 

  Serial.println("WE ARE GOING TO TURN");
  while(abs(imu.getHeading() - target_heading) > 10) {
    imu.updateIMU();
    left_motor_power = TURN_MOTOR_VALUE;
    right_motor_power = TURN_MOTOR_VALUE;
    left_motor.backward(left_motor_power);
    right_motor.forward(right_motor_power);
    Serial.println(abs(imu.getHeading() - target_heading));
  }

  left_motor.stop();
  right_motor.stop();
  
  switch (current_orientation) {
    case (LEFT):
      current_orientation = TOP;
      break;
    case (TOP):
      current_orientation = RIGHT;
      break;
    case (RIGHT):
      current_orientation = BOTTOM;
      break;
    case (BOTTOM):
      current_orientation = LEFT;
      break;
  }

  des_left_offset = getDesLeftDist();
  adjust_setpoint = des_left_offset; // setpoint for PID controller
  adjustController.begin(&adjust_input, &adjust_output, &adjust_setpoint, adjust_p, adjust_i, adjust_d);
  adjustController.start();
  curr_turn = curr_turn + 1;

  heading_offset = heading_offset - 90 < 0 ? heading_offset - 90 + 360 : heading_offset - 90; 
  setState(TILE_FORWARD);

  delay(50);
}

void handleStop() {
  Serial.println("handleStop");
  // slow down 
 
  left_motor_power = SLOW_MOTOR_VALUE;
  right_motor_power = SLOW_MOTOR_VALUE;
  left_motor.backward(left_motor_power);
  right_motor.backward(right_motor_power); 
  
  left_motor.stop();
  right_motor.stop();
  Serial.println("STOP!!!");
}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);

  Serial.println(F("Setting up TOF")); 
  pinMode(left_tof.shutdownPin, OUTPUT);
  pinMode(front_tof.shutdownPin, OUTPUT);
  delay(10);
  
  // all reset   
  digitalWrite(left_tof.shutdownPin, LOW);
  digitalWrite(front_tof.shutdownPin, LOW);
  delay(10);

  // all unreset   
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, HIGH);
  
  delay(10);

  // activating leftTOF and resetting other two
  digitalWrite(left_tof.shutdownPin, HIGH);
  digitalWrite(front_tof.shutdownPin, LOW);
  left_tof.init();

  delay(10);
  Serial.println(F("Set up Left TOF"));

  digitalWrite(front_tof.shutdownPin, HIGH);
  front_tof.init();
  Serial.println(F("Set up front TOF"));

  delay(10);
  Serial.println(F("Set up all TOF"));

  // setup motors and encoders
  left_motor.init();
  right_motor.init();
  
  // then initialize imu
  imu.init();

  adjustController.begin(&adjust_input, &adjust_output, &adjust_setpoint, adjust_p, adjust_i, adjust_d);
  adjustController.setOutputLimits(-255, 255);
  adjustController.setWindUpLimits(-10, 10);
  adjustController.start();

  pinMode(button_pin, INPUT);
  robot_state = INIT;
  current_orientation = BOTTOM;
  current_tile = 0;

  // hardcode first position 
  robot_position.first = 1350;
  robot_position.second = 1650;

  Serial.println(F("Set up all sensors"));
}

void loop() {

  imu.updateIMU();

  Serial.println("Current Tile ");
  Serial.print(current_tile);

//  Serial.println(" ");
//
//  Serial.print("Position: "); 
//  Serial.print(robot_position.first); 
//  Serial.print("   "); 
//  Serial.print(robot_position.second); 
//  Serial.println(" "); 
  
//  Serial.print("Expected left dist: ");
//  Serial.print(des_left_offset);
//  Serial.println(" ");
//
  Serial.print("IMU Heading: "); 
  Serial.print(imu.getHeading()); 
  Serial.println(" ");

  Serial.print("IMU heading offset: "); 
  Serial.print(heading_offset); 
  Serial.println(" ");

  Serial.print("IMU pitch offset: "); 
  Serial.print(pitch_offset); 
  Serial.println(" ");
  Serial.print("pitch: "); 
  Serial.print(imu.getGyroPitch()); 
  Serial.println(" ");
  
  int left_tof_value = left_tof.getDistance(); 
  adjust_input = left_tof_value == -1 ? des_left_offset : left_tof_value;
  adjustController.compute();
  
  // calculate position and localize (match with map)
  calculatePosition(current_orientation, robot_position);
  int next_tile = current_tile + 1;

  if (next_tile >= path.size())  {
    setState(STOP);
  } else {
    // update current tile to next tile if position is in the bounds, update robot path
    updateCurrentTile(robot_position, next_tile, current_tile);
  }

  switch (robot_state) {
    case INIT:
      handleInit();
      break;
    case TILE_FORWARD:
      handleTileForward();
      break;
    case PIT_FORWARD:
      handlePitForward();
      break;
    case TURN_RIGHT:   
      handleTurnRight();
      break;
    case STOP:
     handleStop();
     break;
  }

  delay(50); 
}
