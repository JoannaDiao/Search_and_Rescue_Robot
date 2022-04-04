#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

float mag_hardiron[3];
float mag_softiron[9];

mag_hardiron[0] = 13.61;
mag_hardiron[1] = -22.93;
mag_hardiron[2] = -31.0;

float mag_field = 50.06;

mag_softiron[0] = 0.998;
mag_softiron[1] = -0.002;
mag_softiron[2] = 0.017;
mag_softiron[3] = 0.002;
mag_softiron[4] = 1.071;
mag_softiron[5] = 0.045;
mag_softiron[6] = 0.017;
mag_softiron[7] = 0.045;
mag_softiron[8] = 0.937;

float A_B[3]{   -0.01,   -0.16,    0.11};

float A_Ainv[3][3]
{{  1.02214, -0.00366,  0.00707},
{ -0.00366,  1.01841,  0.00009},
{  0.00707,  0.00009,  0.99862}};

float M_B[3]
{   72.22,  -18.92,   -5.87};

float M_Ainv[3][3]
{{  1.59206,  0.07012,  0.04877},
{  0.07012,  1.30494,  0.04027},
{  0.04877,  0.04027,  1.54240}};

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {0, 0, 0};

// local magnetic declination in degrees
float declination = -14.84;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output


Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

}

void loop()
{

  static int loop_counter = 0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  loop_counter++;
  get_scaled_IMU(&accel, &gyro, &mag, Gxyz, Axya, Mxyz);

  // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

  Mxyz[1] = -Mxyz[1]; //reflect Y and Z
  Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                          Mxyz[0], Mxyz[1], Mxyz[2], deltat);

  if (millis() - lastPrint > PRINT_SPEED) {

    // Define Tait-Bryan angles. Strictly valid only for approximately level movement
    
    // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
    // this code corrects for magnetic declination.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order.
    //
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
          
    // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
    // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
    
    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    //conventional nav, yaw increases CW from North, corrected for local magnetic declination

    yaw = -(yaw + declination);
    if (yaw < 0) yaw += 360.0;
    if (yaw >= 360.0) yaw -= 360.0;

    //     Serial.print("ypr ");
    Serial.print(yaw, 0);
    Serial.print(", ");
    Serial.print(pitch, 0);
    Serial.print(", ");
    Serial.print(roll, 0);
    //          Serial.print(", ");  //prints 49 in 300 ms (~160 Hz) with 8 MHz ATmega328
    //          Serial.print(loop_counter);  //sample & update loops per print interval
    loop_counter = 0;
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
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

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *mag, float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = gyro->gyro.x;
  Gxyz[1] = gyro->gyro.y;
  Gxyz[2] = gyro->gyro.z;

  Axyz[0] = accel->acceleration.x;
  Axyz[1] = accel->acceleration.y;
  Axyz[2] = accel->acceleration.z;
  Mxyz[0] = mag->magnetic.x;
  Mxyz[1] = mag->magnetic.y;
  Mxyz[2] = mag->magnetic.z;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
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

}