void initializeMPU() {
  // Test connection to the MPU
  Wire.beginTransmission(MPU6050_ADDRESS);
  if (Wire.endTransmission() != 0) {
    while (1) {
      Serial.println("ERROR: MPU not connected!");
      delay(1000);
    }
  }

  // Activate the MPU6050 by writing to the PWR_MGMT_1 register
  Wire.beginTransmission(MPU6050_ADDRESS);                     
  Wire.write(0x6B);                                            // PWR_MGMT_1 register
  Wire.write(0x00);                                            // Activate the gyro
  Wire.endTransmission();

  // Set the gyroscope configuration
  Wire.beginTransmission(MPU6050_ADDRESS);                     
  Wire.write(0x1B);                                            // GYRO_CONFIG register
  Wire.write(GYRO_SCALE);                                      // Set the gyro scale
  Wire.endTransmission();

  // Set the accelerometer configuration
  Wire.beginTransmission(MPU6050_ADDRESS);                     
  Wire.write(0x1C);                                            // ACCEL_CONFIG register
  Wire.write(ACCEL_SCALE);                                     // Set the accelerometer scale
  Wire.endTransmission();

  // Configure the digital low-pass filter
  Wire.beginTransmission(MPU6050_ADDRESS);                     
  Wire.write(0x1A);                                            // CONFIG register
  Wire.write(0x03);                                            // Set the DLPF to ~43Hz
  Wire.endTransmission();

  // Get the resting angle (initial pitch and roll)
  readMPUData();
  double total_acceleration_vector = sqrt((accelerometerX * accelerometerX) + (accelerometerY * accelerometerY) + (accelerometerZ * accelerometerZ));
  initial_pitch_angle = asin((float)accelerometerX / total_acceleration_vector) * rad_to_deg;
  initial_roll_angle = asin((float)accelerometerY / total_acceleration_vector) * rad_to_deg;

  Serial.println("MPU Setup completed");
}

void readMPUData() {
  // Start reading data from MPU registers starting at register 0x3B
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);                                             
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 14);

  // Read accelerometer data (high and low bytes)
  int16_t raw_accelerometerX = Wire.read() << 8 | Wire.read();  
  int16_t raw_accelerometerY = Wire.read() << 8 | Wire.read();  
  int16_t raw_accelerometerZ = Wire.read() << 8 | Wire.read();

  // Read temperature data
  int16_t raw_temperature = Wire.read() << 8 | Wire.read();     

  // Read gyroscope data (high and low bytes)
  int16_t raw_gyroscopeX = Wire.read() << 8 | Wire.read();
  int16_t raw_gyroscopeY = Wire.read() << 8 | Wire.read();
  int16_t raw_gyroscopeZ = Wire.read() << 8 | Wire.read();

  // Accelerometer data processing
  accelerometerX = raw_accelerometerX / ACCEL_SCALE_FACTOR;
  accelerometerY = raw_accelerometerY / ACCEL_SCALE_FACTOR;
  accelerometerZ = raw_accelerometerZ / ACCEL_SCALE_FACTOR;

  // Apply correction using calibration errors
  accelerometerX += accelerometer_errorX;
  accelerometerY += accelerometer_errorY;
  accelerometerZ += accelerometer_errorZ;

  // Apply low-pass filter to accelerometer data
  accelerometerX = (1.0 - low_pass_filter_accelerometer) * previous_accelerometerX + low_pass_filter_accelerometer * accelerometerX;
  accelerometerY = (1.0 - low_pass_filter_accelerometer) * previous_accelerometerY + low_pass_filter_accelerometer * accelerometerY;
  accelerometerZ = (1.0 - low_pass_filter_accelerometer) * previous_accelerometerZ + low_pass_filter_accelerometer * accelerometerZ;

  previous_accelerometerX = accelerometerX;
  previous_accelerometerY = accelerometerY;
  previous_accelerometerZ = accelerometerZ;

  // Temperature processing
  temperature = raw_temperature / 340.0 + 36.53;

  // Gyroscope data processing
  gyroscopeX = raw_gyroscopeX / GYRO_SCALE_FACTOR;
  gyroscopeY = raw_gyroscopeY / GYRO_SCALE_FACTOR;
  gyroscopeZ = raw_gyroscopeZ / GYRO_SCALE_FACTOR;

  // Apply correction using calibration errors
  gyroscopeX += gyroscope_errorX;
  gyroscopeY += gyroscope_errorY;
  gyroscopeZ += gyroscope_errorZ;

  // Apply low-pass filter to gyroscope data
  gyroscopeX = (1.0 - low_pass_filter_gyroscope) * previous_gyroscopeX + low_pass_filter_gyroscope * gyroscopeX;
  gyroscopeY = (1.0 - low_pass_filter_gyroscope) * previous_gyroscopeY + low_pass_filter_gyroscope * gyroscopeY;
  gyroscopeZ = (1.0 - low_pass_filter_gyroscope) * previous_gyroscopeZ + low_pass_filter_gyroscope * gyroscopeZ;

  previous_gyroscopeX = gyroscopeX;
  previous_gyroscopeY = gyroscopeY;
  previous_gyroscopeZ = gyroscopeZ;
}
