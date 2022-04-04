#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
Adafruit_ICM20948 icm;

bool init_sensors(void) {
  if (!icm.begin_I2C()) {
    return false;
  }
  accelerometer = icm.getAccelerometerSensor();
  gyroscope = icm.getGyroSensor();
  magnetometer = icm.getMagnetometerSensor();

  return true;
}

void setup_sensors(void) {
  // set lowest range
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  //icm.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  icm.setAccelRateDivisor(4095);
  icm.setGyroRateDivisor(255);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  
  //lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  //lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  //lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
