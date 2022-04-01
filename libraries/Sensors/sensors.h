#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
//#include <queue>

// functions for motors

class Motor {
  public:
    Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin);
    void init();
    void forward(uint16_t pwm_signal);
    void backward(uint16_t pwm_signal);
    void stop();


  private:
    uint16_t speedPin;
    uint16_t forwardPin;
    uint16_t backwardPin;
};

// functions for encoders

class Encoder {
  public:
    Encoder(uint16_t clock_pin, uint16_t dt_pin);
    void init();

    // this functions should get called on every loop to update the encoder values
    void update();

    // gives absolute position of encoder
    unsigned long getTicks();

  private:
    uint16_t clockPin;
    uint16_t dtPin;
    uint32_t encoderPosCount;
    uint32_t lastValue;
    unsigned long ticks;
    unsigned long lastDebounceTime;
};


// functions for imu sensor
class IMU {
  public:
    IMU() {}
    void init();
    // need to define CALIBRATE_IMU to use this function
    void calibrate(float* mag_hardiron, float* mag_softiron, float mag_field, int num_points);

    // this function should get called on every loop to update the IMU measurement
    void update();

    // test function for accel and gyro fusion
    void updateIMU();
    
    float getGyroPitch();

    float getHeading();
    float getPitch();
    float getRoll();
    void getQuaternion(float* qw, float* qx, float* qy, float* qz);
    void getYPR(float* y, float* p, float* r);

  private:
    float heading;
    float pitch;
    float roll;

    float _qx, _qy, _qz, _qw;
    // Vector to hold integral error for Mahony method
    float eInt[3];
    float q[4];
    unsigned long now, last; //micros() timers for AHRS loop
    float deltat;  //loop time in seconds

    void calibrateTry(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *mag, float Axyz[3], float Gxyz[3], float Mxyz[3]);
    void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

    Adafruit_ICM20948 icm;

    //Adafruit_NXPSensorFusion filter; // slowest
    //Adafruit_Madgwick filter;  // faster than NXP
    Adafruit_Mahony filter;  // fastest/smalleset
    Adafruit_Sensor_Calibration_EEPROM cal;
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
};



// functions for TOF sensor
class TOF {
  public:
    TOF(uint16_t lox_address, uint16_t shutdown_pin, bool is_left);
    void init();
    int getDistance();
    uint16_t shutdownPin;
    
  private:
    Adafruit_VL53L0X lox;
    bool isLeft;
    uint32_t buffer[10];
    int distance;
    int lastTime;
    uint16_t loxAddress;
};

class LeftTOF {
	public:
	    LeftTOF(uint16_t lox_address, uint16_t shutdown_pin);
	    void init();
	    int getDistance();
	    uint16_t shutdownPin;
	    //std::queue<uint16_t> buffer_;
	    void addValue();
        uint16_t getValue();
        void clearValues();
        bool shouldAdjustRight();
        bool shouldAdjustLeft();
    
  	private:
	    Adafruit_VL53L0X lox;
	    uint32_t buffer[10];
	    int distance;
	    uint16_t loxAddress;
};

#endif
