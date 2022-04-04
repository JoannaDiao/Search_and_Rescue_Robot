#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <ArduinoSTL.h>
#include <sensors.h>
#include <helpers.h>
#include <HashMap.h>
#include <microTuple.h>
#include <map>

#define LMotorForwardPin 1
#define LMotorBackwardPin 2
#define LMotorSpeedPin 3
#define RMotorForwardPin 4
#define RMotorBackwardPin 5
#define RMotorSpeedPin 6

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

// IO
struct {
  // Left Motor connections
  const int ENA = 9;
  const int IN1 = 8;
  const int IN2 = 7;
  // Right Motor connections
  const int ENB = 3;
  const int IN3 = 5;
  const int IN4 = 4;
} IO;

robot_state_t robot_state;
int button_state = 0;
const int button_pin = 2;
int current_tile;
robot_orientation current_orientation;
std::pair<double, double> robot_position;
const double WIDTH = 1800;
const double TILE_WIDTH = 300;
int left_motor_power = 0;
int right_motor_power = 0;
double prev_pitch = 0;
const int SAND_MOTOR_VALUE = 60;
const int PIT_MOTOR_LOW = 40;
const int PIT_MOTOR_MED = 60;
const int PIT_MOTOR_HIGH = 70;
const int TILE_MOTOR_VALUE = 60;
const int TURN_MOTOR_VALUE_LEFT = 40;
const int TURN_MOTOR_VALUE_RIGHT = 40; // either stop (speed 0) or 50 in the reverse direction
const int ROBOT_WIDTH = 80;
const int ROBOT_LENGTH = 160;
const int ROBOT_MOTOR_OFFSET = 5;
const int TRAP_EXIT_TIME = 20000;
const double CENTER_TILE_TOL = 10;
const int PIT_DELAY_LOW = 30000;
const int PIT_DELAY_MED = 50000;
const int PIT_DELAY_HIGH = 30000;
const double PITCH_UPWARDS_VALUE = 40;
const double PITCH_DOWNWARDS_VALUE = -40;
const double ADJUST_VALUE = 5;
LeftTofSensor left_tof;
TOF back_tof;
TOF front_tof;
Motor left_motor(LMotorSpeedPin, LMotorForwardPin, LMotorBackwardPin);
Motor right_motor(RMotorSpeedPin, RMotorForwardPin, RMotorBackwardPin);
IMU imu;


// S = start, E = end, T = turn
std::map<std::pair<int, int>, char> course = {
  {{5, 3}, 'S'}, {{5, 0}, 'T'}, {{0, 0}, 'T'}, {{0, 5}, 'T'},
  {{4, 5}, 'T'}, {{4, 1}, 'T'}, {{1, 1}, 'T'}, {{1, 4}, 'T'},
  {{3, 4}, 'T'}, {{3, 2}, 'T'}, {{2, 2}, 'T'}, {{2, 3}, 'E'}
};

// F = front tof, B = back tof, x and y in mm, based on 1.8 m
std::vector<std::vector<MicroTuple<int, int, char>>> coords = {
  {{150, 150, 'F'}, {450, 150, 'B'}, {750, 150, 'B'}, {1050, 150, 'F'}, {1350, 150, 'F'}, {1650, 150, 'F'}},
  {{150, 450, 'F'}, {450, 450, 'F'}, {750, 450, 'B'}, {1050, 450, 'F'}, {1350, 450, 'F'}, {1650, 450, 'B'}},
  {{150, 750, 'F'}, {450, 750, 'F'}, {750, 750, 'F'}, {1050, 750, 'F'}, {1350, 750, 'B'}, {1650, 750, 'B'}},
  {{150, 1050, 'B'}, {450, 1050, 'B'}, {750, 1050, 'F'}, {1050, 1050, 'B'}, {1350, 1050, 'F'}, {1650, 1050, 'F'}},
  {{150, 1350, 'B'}, {450, 1350, 'F'}, {750, 1350, 'F'}, {1050, 1350, 'B'}, {1350, 1350, 'B'}, {1650, 1350, 'F'}},
  {{150, 1650, 'F'}, {450, 1650, 'F'}, {750, 1650, 'F'}, {1050, 1650, 'B'}, {1350, 1650, 'B'}, {1650, 1650, 'F'}}
};

std::vector<std::pair<int, int>> path = {
  {5, 3}, {5, 2}, {5, 1}, {5, 0}, {4, 0}, {3, 0},
  {2, 0}, {1, 0}, {0, 0}, {0, 1}, {0, 2}, {0, 3},
  {0, 4}, {0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5},
  {4, 4}, {4, 3}, {4, 2}, {4, 1}, {3, 1}, {2, 1},
  {1, 1}, {1, 2}, {1, 3}, {1, 4}, {2, 4}, {3, 4},
  {3, 3}, {3, 2}, {2, 2}, {2, 3}
};


/* ------------ Functions --------------- */

void setState(robot_state_t new_state) {
  robot_state = new_state;
}

void calculatePosition(robot_orientation current_orientation, std::pair<double, double>& position) {
  // obtain TOF data, front_tof, back_tof, left_tof



  MicroTuple<int, int, char> coord = coords[path[current_tile].first][path[current_tile].second];
  char which_tof = coord.get<2>();

  switch (current_orientation) {
    case LEFT:
      position.first = left_tof.getValue();
      position.second = which_tof == 'B' ? WIDTH - back_tof.getDistance() : front_tof.getDistance();
      break;

    case RIGHT:
      position.first = WIDTH - left_tof.getValue();
      position.second = which_tof == 'B' ? back_tof.getDistance() : WIDTH - front_tof.getDistance();
      break;

    case TOP:
      position.first = which_tof == 'B' ? back_tof.getDistance() : WIDTH - front_tof.getDistance();
      position.second = left_tof.getValue();
      break;

    case BOTTOM:
      position.first = which_tof == 'B' ? WIDTH - back_tof.getDistance() : front_tof.getDistance();
      position.second = WIDTH - left_tof.getValue();
      break;
  }
}

void updateCurrentTile(const std::pair<double, double>& position, const int next_tile, int& current_tile) {
  MicroTuple<int, int, char> coord = coords[path[next_tile].first][path[next_tile].second];
  double next_center_x = coord.get<0>();
  double next_center_y = coord.get<1>();
  double position_x = position.first;
  double position_y = position.second;

  if (position_x <= next_center_x + TILE_WIDTH / 2.0 && position_x > next_center_x - TILE_WIDTH / 2.0
      && position_y <= next_center_y + TILE_WIDTH / 2.0 && position_y > next_center_y - TILE_WIDTH / 2.0) { // this may need tolerances
    current_tile = next_tile;
  }
}

// INIT state, robot waits for push button to be pressed before moving
void handleInit() {
  while (digitalRead(button_pin) == LOW) {}
  setState(TILE_FORWARD);
}

void handleTileForward() {
  left_tof.addValue();

  driveStraight();

  if ((imu.getPitch() - prev_pitch) < PITCH_DOWNWARDS_VALUE) {
    setState(PIT_FORWARD);
  }

  // check for turn or stop
  // only change state when robot is center of tile
  std::map<std::pair<int, int>, char>::iterator it = course.find(path[current_tile]);
  if (it != course.end()) {
    char landmark = it->second;

    if (landmark == 'T') {
      // only change state when robot is center of tile
      // double center_x =  std::get<0>(coords[path[current_tile][0]][path[current_tile][1]]);
      // double center_y =  std::get<1>(coords[path[current_tile][0]][path[current_tile][1]]);
      // if (robot_position.first > center_x - CENTER_TILE_TOL && robot_position.first < center_x + CENTER_TILE_TOL)
      delay(TURN_DELAY); // to get to the center of turn tile. if doesnt work use front tof -- do math
      setState(TURN_RIGHT);
    } else if (landmark == 'E') {
      delay(TURN_DELAY);
      setState(STOP);
    }
  }

  if (left_tof.shouldAdjustLeft()) {
    setState(LEFT_ADJUST);
  }

  if (left_tof.shouldAdjustRight()) {
    setState(RIGHT_ADJUST);
  }
}

void handlePitForward() {
  left_motor_power = PIT_MOTOR_LOW;
  right_motor_power = PIT_MOTOR_LOW;
  left_motor.forward(left_motor_power);
  right_motor.forward(right_motor_power);

  left_motor_power = PIT_MOTOR_MED;
  right_motor_power = PIT_MOTOR_MED;
  left_motor.forward(left_motor_power);
  right_motor.forward(right_motor_power);

  left_motor_power = PIT_MOTOR_HIGH;
  right_motor_power = PIT_MOTOR_HIGH;
  left_motor.forward(left_motor_power);
  right_motor.forward(right_motor_power);

  // exit pit state
  setState(TILE_FORWARD);
}

void handleTurnRight() {
  int target_heading = imu.getHeading() - 90 < 0 ? imu.getHeading() - 90 + 360 : imu.getHeading() - 90; 

  left_motor.stop();
  right_motor.stop();
  
  while (abs(imu.getHeading() - target_heading) <= 90) {
    left_motor_power = TURN_MOTOR_VALUE_LEFT;
    right_motor_power = TURN_MOTOR_VALUE_RIGHT;
    left_motor.forward(left_motor_power);
    right_motor.backward(right_motor_power);
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

  setState(TILE_FORWARD);
  left_tof.clearValues();
}

void handleStop() {
  left_motor.stop();
  right_motor.stop();
  setState(INIT);
}

void setup() {
  pinMode(button_pin, INPUT);
  robot_state = INIT;
  current_orientation = BOTTOM;
  current_tile = 0;

}

void loop() {
  // calculate position and localize (match with map)
  calculatePosition(current_orientation, robot_position);
  int next_tile = current_tile + 1;

  if (next_tile >= path.size())  {
    setState(STOP);
  } else {
    // update current tile to next tile if position is in the bounds, update robot path
    updateCurrentTile(robot_position, next_tile, current_tile);
  }

  prev_pitch = imu.getPitch();

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
    case LEFT_ADJUST:
      handleLeftAdjust();
      break;
    case RIGHT_ADJUST:
      handleRightAdjust();
      break;
    case STOP:
      handleStop();
      break;
  }
}