void printRawMPUData() {
  Serial.print("Gyroscope X: ");
  Serial.print(gyroscopeX);
  Serial.print(", ");
  Serial.print("Gyroscope Y: ");
  Serial.print(gyroscopeY);
  Serial.print(", ");
  Serial.print("Gyroscope Z: ");
  Serial.print(gyroscopeZ);
  Serial.print(", ");
  Serial.print("Accelerometer X: ");
  Serial.print(accelerometerX);
  Serial.print(", ");
  Serial.print("Accelerometer Y: ");
  Serial.print(accelerometerY);
  Serial.print(", ");
  Serial.print("Accelerometer Z: ");
  Serial.print(accelerometerZ);
}

void printRollAndPitch() {
  Serial.print("Kalman Pitch: ");
  Serial.print(pitch_angle[1]);
  Serial.print(", ");
  Serial.print("Kalman Roll: ");
  Serial.print(roll_angle[1]);
}

void printExecutionTimes() {
  Serial.print("Complementary Filter Time: ");
  Serial.print(execution_time[0]);
  Serial.print(", ");
  Serial.print("Kalman Filter Time: ");
  Serial.print(execution_time[1]);
  Serial.print(", ");
  Serial.print("Madgwick Filter Time: ");
  Serial.print(execution_time[2]);
  Serial.print(", ");
  Serial.print("MPU Read Time: ");
  Serial.print(data_fetching_time);
}

void printMaxExecutionTimes() {
  Serial.print("Max Complementary Filter Time: ");
  Serial.print(max_execution_time[0]);
  Serial.print(", ");
  Serial.print("Max Kalman Filter Time: ");
  Serial.print(max_execution_time[1]);
  Serial.print(", ");
  Serial.print("Max Madgwick Filter Time: ");
  Serial.print(max_execution_time[2]);
  Serial.print(", ");
  Serial.print("Max MPU Read Time: ");
  Serial.print(max_data_fetching_time);
}

void printDeltaTime() {
  Serial.print("Delta Time (dt): ");
  Serial.print(delta_time);
}

void regulateLoopRate(int frequency) {
  // Regulate loop rate to the specified frequency (Hz)
  float inverse_frequency = 1.0 / frequency * 1000000.0;
  unsigned long current_check_time = micros();

  // Wait until the time passed equals the desired period
  while (inverse_frequency > (current_check_time - current_time_microseconds)) {
    current_check_time = micros();
  }
}

float fastInverseSqrt(float x) {
  // Fast inverse square root for Madgwick filter
  return 1.0 / sqrtf(x); 
}
