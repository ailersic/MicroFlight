
//========================================================================================================================//
//                                                 ADDRESSES AND SCALES                                                   //
//========================================================================================================================//

// MPU6050 addresses
const int MPU6050_ADDR = 0x68; // MPU6050 I2C address
const int ACCEL_XOUT_H = 0x3B; // Register address for accelerometer X high byte
const int GYRO_XOUT_H = 0x43;  // Register address for gyroscope X high byte

// MPU6050 scale factors
const float ACCEL_SCALE = 16384.0; // LSB sensitivity for accelerometer (+/- 2g)
const float GYRO_SCALE = 131.0;    // LSB sensitivity for gyroscope (+/- 250 degrees/sec)

// QMC5883L addresses
const int QMC5883L_ADDR = 0x0D; // QMC5883L I2C address
const int MAG_XOUT_H = 0x00; // Register address for magnetometer X high byte

// QMC5883L scale factor
const float MAG_SCALE = 12000.0; // LSB sensitivity for magnetometer (+/- 2 Gauss)

// BMP180 addresses
const int BMP180_ADDR = 0x77; // BMP180 I2C address
const int BARO_OUT_H = 0x03; // Register address for barometer high byte

// BMP180 scale factor
const float BARO_SCALE = 1.0; // LSB sensitivity for barometer (+/- something kPa)

//========================================================================================================================//
//                                              CHECKING SENSORS AVAILABLE                                                //
//========================================================================================================================//

void enableBypass() {
  // MPU6050 has an aux I2C port, so we want to enable access via it
  delay(500);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission(true);
}

void checkSensors() {
  byte error, address;

  Serial.println("Scanning...");
  for(address = 1; address < 127; address++ ) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      switch (address) {
        case MPU6050_ADDR:
          Serial.println("MPU6050 found!");
          MPU6050_avail = true;
          break;
        case QMC5883L_ADDR:
          Serial.println("QMC5883L found!");
          QMC5883L_avail = true;
          break;
        case BMP180_ADDR:
          Serial.println("BMP180 found!");
          BMP180_avail = true;
          break;
        default:
          Serial.print("Unknown device at ");
          if (address<16) 
            Serial.print("0");
          Serial.println(address,HEX);
      }
    }  
  }
  Serial.println("done\n");
}

//========================================================================================================================//
//                                                ACCELEROMETER/GYROSCOPE                                                 //
//========================================================================================================================//

void setupAccelGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);     // Wake up MPU6050
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // Gyro config register
  Wire.write(0x00);     // range +/- 250 deg/s
  Wire.endTransmission(true);

  delay(500);
  setAccelGyroRef();
}

void setAccelGyroRef() {
  getAccelGyro();
  //aX0 = aX;
  //aY0 = aY;
  //aZ0 = aZ;

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

  float a = 0.99;

  // Convert to float
  aX = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aX0) + (1 - a)*aX;
  aY = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aY0) + (1 - a)*aY;
  aZ = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / ACCEL_SCALE - aZ0) + (1 - a)*aZ;

  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  // Convert to float
  gX = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gX0) + (1 - a)*gX;
  gY = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gY0) + (1 - a)*gY;
  gZ = a*(((int16_t) (Wire.read() << 8 | Wire.read())) / GYRO_SCALE - gZ0) + (1 - a)*gZ;
}

//========================================================================================================================//
//                                                     MAGNETOMETER                                                       //
//========================================================================================================================//

void setupMag() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09);  // mode register
  Wire.write(0x0D);     // continuous measurement
  Wire.endTransmission(true);

  delay(500);
  setMagRef();
}

void setMagRef() {
  getMag();
}

void getMag() {
  // Read magnetometer data
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(MAG_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883L_ADDR, 6, true);

  // Convert to float
  float raw_mX = ((int16_t) (Wire.read() | Wire.read() << 8)) / MAG_SCALE;
  float raw_mY = ((int16_t) (Wire.read() | Wire.read() << 8)) / MAG_SCALE;
  float raw_mZ = ((int16_t) (Wire.read() | Wire.read() << 8)) / MAG_SCALE;

  if (raw_mX > mXul || mXul == 0) mXul = raw_mX; if (raw_mX < mXll || mXll == 0) mXll = raw_mX;
  if (raw_mY > mYul || mYul == 0) mYul = raw_mY; if (raw_mY < mYll || mYll == 0) mYll = raw_mY;
  if (raw_mZ > mZul || mZul == 0) mZul = raw_mZ; if (raw_mZ < mZll || mZll == 0) mZll = raw_mZ;
  
  float eps = 0.0001;
  float a = 0.99;

  mX = a*(-1.0 + 2*(raw_mX - mXll) / (mXul - mXll + eps)) + (1 - a)*mX;
  mY = a*(-1.0 + 2*(raw_mY - mYll) / (mYul - mYll + eps)) + (1 - a)*mY;
  mZ = a*(-1.0 + 2*(raw_mZ - mZll) / (mZul - mZll + eps)) + (1 - a)*mZ;
}

//========================================================================================================================//
//                                                      BAROMETER                                                         //
//========================================================================================================================//

void setupBaro() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0x09);  // mode register
  Wire.write(0x1D);     // continuous measurement
  Wire.endTransmission(true);

  delay(500);
  setBaroRef();
}

void setBaroRef() {
  getBaro();
  pres0 = pres;
}

void getBaro() {
  // Read magnetometer data
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(BARO_OUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDR, 2, true);

  // Convert to float
  pres = ((int16_t) (Wire.read() << 8 | Wire.read())) / BARO_SCALE - pres0;
}