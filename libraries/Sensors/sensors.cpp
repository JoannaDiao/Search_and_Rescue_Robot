#include "Arduino.h"
#include "sensors.h"
#include <Wire.h>

#define MAX_PWM 255
#define DEBOUNCE_DELAY 50  // 50 ms (may need to change)

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

// uncomment this for debugging
//#define DEBUG_ON

// uncomment this for calibration
//#define CALIBRATE_IMU

Motor::Motor(uint16_t speed_pin, uint16_t forward_pin, uint16_t backward_pin)
{
    speedPin = speed_pin;
    forwardPin = forward_pin;
    backwardPin = backward_pin;
}

void Motor::init()
{
    pinMode(speedPin, OUTPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
#ifdef DEBUG_ON
    Serial.println(F("Initialized Motor"));
#endif
}

void Motor::forward(uint16_t pwm_signal = 100)
{
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
}

void Motor::backward(uint16_t pwm_signal = 100)
{
    uint16_t pwm = map(pwm_signal, 0, 100, 0, MAX_PWM);
    analogWrite(speedPin, pwm);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
}

void Motor::stop()
{
    analogWrite(speedPin, MAX_PWM);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

Encoder::Encoder(uint16_t clock_pin, uint16_t dt_pin)
{
    clockPin = clock_pin;
    dtPin = dt_pin;
    lastDebounceTime = 0;
    encoderPosCount = 0;
}

void Encoder::init()
{
    pinMode(clockPin, INPUT);
    pinMode(dtPin, INPUT);
    lastValue = digitalRead(clockPin);
#ifdef DEBUG_ON
    Serial.println(F("Initialized Encoder"));
#endif
}

void Encoder::update()
{
    int clockVal = digitalRead(clockPin);

    if (clockVal != lastValue) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    // check if reading is valid and if so check direction of encoder
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // if dtPin is different then clockVal we are rotating forward
        if (digitalRead(dtPin) != clockVal) {
            ++encoderPosCount;
        } else {
            --encoderPosCount;
        }
    }

    lastValue = clockVal;
}

unsigned long Encoder::getTicks()
{
    return encoderPosCount;
}

TOF::TOF(uint16_t lox_address, uint16_t shutdown_pin, bool is_left = false)
{
    loxAddress = lox_address;
    shutdownPin = shutdown_pin;
    isLeft = is_left;
}
void TOF::init()
{
    Adafruit_VL53L0X::VL53L0X_Sense_config_t config = isLeft ? Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT:Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE;
    if (!lox.begin(loxAddress, false, &Wire, config)) {
        Serial.println(F("Failed to boot VL53L0X"));
        while (1);
    }
    if (!isLeft) {
    	lox.startRangeContinuous();
	}
#ifdef DEBUG_ON
    Serial.println(F("Found VL53L0X"));
#endif
}

int TOF::getDistance()
{
	uint16_t measure = -1;
	if (!isLeft) {

	    if (lox.isRangeComplete()) {
	        // ignore greater than 1m for left sensor and 2m for front
	        uint16_t MAX_D = isLeft ? 1000 : 2000;
	
	        measure = lox.readRangeResult();
	        distance = measure > MAX_D ? -1 : measure;
	        lastTime = micros();
	    }
	
	    return distance;
	} else {
		measure = lox.readRange();
		
	    return measure > 1000 ? -1 : measure;
	}
}

void IMU::init()
{
    if (!icm.begin_I2C()) {
        Serial.println("Failed to find imu");
        while (1) delay(10);
    }
    icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

    // set slightly above refresh rate
    icm.setAccelRateDivisor(4095);
    icm.setGyroRateDivisor(255);
    icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);

    accelerometer = icm.getAccelerometerSensor();
    gyroscope = icm.getGyroSensor();
    magnetometer = icm.getMagnetometerSensor();

    deltat = 0;
    now = 0;
    last = 0;

    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    eInt[0] = 0.0;
    eInt[1] = 0.0;
    eInt[2] = 0.0;


    // if (!cal.begin()) {
    //     Serial.println("Failed to initialize calibration helper");
    //     while (1) yield();
    // }
    // if (!cal.loadCalibration()) {
    //     Serial.println("No calibration loaded/found... will start with defaults");
    // } else {
    //     Serial.println("Loaded existing calibration");
    // }
}

void IMU::calibrate(float* mag_hardiron, float* mag_softiron, float mag_field, int num_points = 100)
{
#ifdef CALIBRATE_IMU
    sensors_event_t mag_event, gyro_event, accel_event;
    
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
    
    Serial.println(F("Adafruit AHRS - IMU Calibration!"));

    Serial.println("Calibration filesys test");
    if (!cal.begin()) {
        Serial.println("Failed to initialize calibration helper");
        while (1) yield();
    }
    if (! cal.loadCalibration()) {
        Serial.println("No calibration loaded/found... will start with defaults");
    } else {
        Serial.println("Loaded existing calibration");
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();
    
    Wire.setClock(400000); // 400KHz

    if (num_points < 100)
        num_points = 100;

    
    float accelx_u = 0, accely_u = 0, accelz_u = 0;
    float gx_u = 0, gy_u = 0, gz_u = 0;

    // read num_points imu measurements and get the standard deviation then apply the offset
    for (int i = 0; i < num_points; ++i)
    {
        magnetometer->getEvent(&mag_event);
        gyroscope->getEvent(&gyro_event);
        accelerometer->getEvent(&accel_event);

        accelx_u += accel_event.acceleration.x;
        accely_u += accel_event.acceleration.y;
        accelz_u += accel_event.acceleration.z;

        gx_u += gyro_event.gyro.x;
        gy_u += gyro_event.gyro.y;
        gz_u += gyro_event.gyro.z;
        delay(20);
    }

    cal.accel_zerog[0] = accelx_u / num_points;
    cal.accel_zerog[1] = accely_u / num_points;
    cal.accel_zerog[2] = accelz_u / num_points - 9.81;
    
    cal.gyro_zerorate[0] = gx_u / num_points;
    cal.gyro_zerorate[1] = gy_u / num_points;
    cal.gyro_zerorate[2] = gz_u / num_points;
    
    cal.mag_hardiron[0] = mag_hardiron[0];
    cal.mag_hardiron[1] = mag_hardiron[1];
    cal.mag_hardiron[2] = mag_hardiron[2];

    cal.mag_field = mag_field;
    
    cal.mag_softiron[0] = mag_softiron[0];
    cal.mag_softiron[1] = mag_softiron[1];
    cal.mag_softiron[2] = mag_softiron[2];
    cal.mag_softiron[3] = mag_softiron[3];
    cal.mag_softiron[4] = mag_softiron[4];
    cal.mag_softiron[5] = mag_softiron[5];
    cal.mag_softiron[6] = mag_softiron[6];
    cal.mag_softiron[7] = mag_softiron[7];
    cal.mag_softiron[8] = mag_softiron[8];

    if (! cal.saveCalibration()) {
        Serial.println("**WARNING** Couldn't save calibration");
    } else {
        Serial.println("Wrote calibration");    
    }
    cal.printSavedCalibration()
#endif
}

float IMU::getGyroPitch()
{
	sensors_event_t gyro_event;
	gyroscope->getEvent(&gyro_event);
	
	return gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

void IMU::calibrateTry(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *mag, float Axyz[3], float Gxyz[3], float Mxyz[3])
{
    float A_B[3]                         
    {    0.01,   -0.14,    0.12};      
                                    
    float A_Ainv[3][3]                   
    {{  1.01460, -0.01535,  0.00246}, 
    { -0.01535,  1.01383, -0.00169},  
    {  0.00246, -0.00169,  1.00267}};

    float M_B[3]
    {   72.22,  -18.92,   -5.87};

    float M_Ainv[3][3]
    {{  1.59206,  0.07012,  0.04877},
    {  0.07012,  1.30494,  0.04027},
    {  0.04877,  0.04027,  1.54240}};

    float temp[3];

    Gxyz[0] = gyro->gyro.x * SENSORS_RADS_TO_DPS;
    Gxyz[1] = gyro->gyro.y * SENSORS_RADS_TO_DPS;
    Gxyz[2] = gyro->gyro.z * SENSORS_RADS_TO_DPS;

    Axyz[0] = accel->acceleration.x;
    Axyz[1] = accel->acceleration.y;
    Axyz[2] = accel->acceleration.z;
    Mxyz[0] = mag->magnetic.x;
    Mxyz[1] = mag->magnetic.y;
    Mxyz[2] = mag->magnetic.z;

    //apply accel offsets (bias) and scale factors from Magneto

    for (int i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
    Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
    Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
    Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
    vector_normalize(Axyz);

    //apply mag offsets (bias) and scale factors from Magneto

    for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
    Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
    Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
    Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
    vector_normalize(Mxyz);

    Mxyz[1] = -Mxyz[1]; //reflect Y and Z
    Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void IMU::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, hz;  //observed West horizon vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Measured horizon vector = a x m (in body frame)
    hx = ay * mz - az * my;
    hy = az * mx - ax * mz;
    hz = ax * my - ay * mx;
    // Normalise horizon vector
    norm = sqrt(hx * hx + hy * hy + hz * hz);
    if (norm == 0.0f) return; // Handle div by zero

    norm = 1.0f / norm;
    hx *= norm;
    hy *= norm;
    hz *= norm;

    // Estimated direction of Up reference vector
    ux = 2.0f * (q2q4 - q1q3);
    uy = 2.0f * (q1q2 + q3q4);
    uz = q1q1 - q2q2 - q3q3 + q4q4;

    // estimated direction of horizon (West) reference vector
    wx = 2.0f * (q2q3 + q1q4);
    wy = q1q1 - q2q2 + q3q3 - q4q4;
    wz = 2.0f * (q3q4 - q1q2);

    // Error is the summed cross products of estimated and measured directions of the reference vectors
    // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

    ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
    ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
    ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }


    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;


    //update quaternion with integrated contribution
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    gx = gx * (0.5*deltat); // pre-multiply common factors
    gy = gy * (0.5*deltat);
    gz = gz * (0.5*deltat);
    float qa = q1;
    float qb = q2;
    float qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void IMU::update()
{
    sensors_event_t accel, gyro, mag;

    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    float Axyz[3], Gxyz[3], Mxyz[3];

    calibrateTry(&accel, &gyro, &mag, Axyz, Gxyz, Mxyz);

    // Update the SensorFusion filter
    // filter.update(Gxyz[0], Gxyz[1], Gxyz[2], 
    //             Axyz[0], Axyz[1], Axyz[2], 
    //             Mxyz[0], Mxyz[1], Mxyz[2]);
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                          Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    heading   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    heading   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    //conventional nav, heading increases CW from North, corrected for local magnetic declination
    float declination = 0.0;
    heading = -(heading + declination);
    if (heading < 0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;
    
#ifdef DEBUG_ON
    Serial.print("Raw: ");
    Serial.print(accel.acceleration.x, 4); Serial.print(", ");
    Serial.print(accel.acceleration.y, 4); Serial.print(", ");
    Serial.print(accel.acceleration.z, 4); Serial.print(", ");
    Serial.print(gx, 4); Serial.print(", ");
    Serial.print(gy, 4); Serial.print(", ");
    Serial.print(gz, 4); Serial.print(", ");
    Serial.print(mag.magnetic.x, 4); Serial.print(", ");
    Serial.print(mag.magnetic.y, 4); Serial.print(", ");
    Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

    // roll = filter.getRoll();
    // pitch = filter.getPitch();
    // heading = filter.getYaw();

    // filter.getQuaternion(&_qw, &_qx, &_qy, &_qz);
}

void IMU::updateIMU()
{
    sensors_event_t accel, gyro, mag;

    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    float Axyz[3], Gxyz[3], Mxyz[3];

    calibrateTry(&accel, &gyro, &mag, Axyz, Gxyz, Mxyz);

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    filter.updateIMU(Gxyz[0], Gxyz[1], Gxyz[2], Axyz[0], Axyz[1], Axyz[2], deltat);

    heading = filter.getYaw();
    pitch = filter.getPitch();
    roll = filter.getRoll();

    filter.getQuaternion(&q[0], &q[1], &q[2], &q[3]);
}

float IMU::getPitch()
{
    return pitch;
}

float IMU::getHeading()
{
    return heading;
}

float IMU::getRoll()
{
    return roll;
}

void IMU::getQuaternion(float *qw, float *qx, float *qy, float *qz)
{
    *qw = q[0];
    *qx = q[1];
    *qy = q[2];
    *qz = q[3];
}

void IMU::getYPR(float *y, float *p, float *r)
{
    *y = heading;
    *p = pitch;
    *r = roll;
}
