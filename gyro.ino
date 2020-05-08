// Functions for handling communication with the mpu
int gyro_raw_roll, gyro_raw_pitch, gyro_raw_yaw;
double gyro_offset_roll, gyro_offset_pitch, gyro_offset_yaw;

void setupGyro() {
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

// calculate average over 100 samples to define a calibration offset
void calibrateGyro() {
  for (int i = 0; i < 100; i++) {
    readGyro();
    gyro_offset_roll += gyro_raw_roll;
    gyro_offset_pitch += gyro_raw_pitch;
    gyro_offset_yaw += gyro_raw_yaw;
    delay(10);
  }
  gyro_offset_roll /= 100;
  gyro_offset_pitch /= 100;
  gyro_offset_yaw /= 100;
}

void readGyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // starting with register 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);  // request a total of 6 registers (false is faster?)
  gyro_raw_roll = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro_raw_pitch = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro_raw_yaw = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gyro_roll = gyro_roll * (1 - gyro_credibility) + double(gyro_raw_roll - gyro_offset_roll) / 65.5 * gyro_credibility;     // convert readings to deg/s & pseudo fitering to give the latest reading less credibility
  gyro_pitch = gyro_pitch * (1 - gyro_credibility) + double(gyro_raw_pitch - gyro_offset_pitch) / 65.5 * gyro_credibility;
  gyro_yaw = gyro_yaw * (1 - gyro_credibility) + double(gyro_raw_yaw - gyro_offset_yaw) / 65.5 * gyro_credibility;
}
