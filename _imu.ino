/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/

const int MPU6050_ADDR = 0x68; // MPU6050 I2C address
const int ACCEL_XOUT_H = 0x3B; // Register address for accelerometer X high byte
const int GYRO_XOUT_H = 0x43;  // Register address for gyroscope X high byte

// MPU6050 scale factors
const float ACCEL_SCALE = 16384.0; // LSB sensitivity for accelerometer (+/- 2g)
const float GYRO_SCALE = 131.0;    // LSB sensitivity for gyroscope (+/- 250 degrees/sec)

void setupIMU() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission(true);

  setAccelGyroRef();
}

void setAccelGyroRef() {
  getAccelGyro();
  aX0 = aX;
  aY0 = aY;
  aZ0 = aZ - 1.0;
  gX0 = gX;
  gY0 = gY;
  gZ0 = gZ;
}

void getAccelGyro() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  // Convert to float
  aX = ((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aX0;
  aY = ((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aY0;
  aZ = ((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aZ0;

  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  // Convert to float
  gX = ((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gX0;
  gY = ((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gY0;
  gZ = ((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gZ0;
}
