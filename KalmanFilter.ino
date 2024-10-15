void initializeKalmanFilter() {
  // Initialize angle estimates to the initial pitch and roll
  pitch_angle[1] = initial_pitch_angle;
  roll_angle[1] = initial_roll_angle;

  // Initialize bias estimates for gyroscope
  gyroscope_biasX = 0.0;
  gyroscope_biasY = 0.0;

  // Initialize the error covariance matrix
  for (int i = 0; i < 2; i++) {
    error_covariance[i][0][0] = 0.0;
    error_covariance[i][0][1] = 0.0;
    error_covariance[i][1][0] = 0.0;
    error_covariance[i][1][1] = 0.0;
  }
}

float updateKalmanFilter(float previousAngle, float currentAngle, float currentRate, float deltaTime, int axis) {
  float unbiasedRate = currentRate - gyroscope_biasX;  // Unbiased rate of change for the specified axis
  float predictedAngle = previousAngle + deltaTime * unbiasedRate;  // Predicted angle

  // Update the error covariance matrix
  error_covariance[axis][0][0] += deltaTime * (deltaTime * error_covariance[axis][1][1] - error_covariance[axis][0][1] - error_covariance[axis][1][0] + kalman_Q_angle);
  error_covariance[axis][0][1] -= deltaTime * error_covariance[axis][1][1];
  error_covariance[axis][1][0] -= deltaTime * error_covariance[axis][1][1];
  error_covariance[axis][1][1] += kalman_Q_bias * deltaTime;

  // Compute Kalman gain
  float innovationCovariance = error_covariance[axis][0][0] + kalman_R_measure;
  float kalmanGain[2];  // Kalman gain
  kalmanGain[0] = error_covariance[axis][0][0] / innovationCovariance;
  kalmanGain[1] = error_covariance[axis][1][0] / innovationCovariance;

  // Update the estimate with the new measurement
  float angleDifference = currentAngle - predictedAngle;  // Angle difference
  predictedAngle += kalmanGain[0] * angleDifference;
  gyroscope_biasX += kalmanGain[1] * angleDifference;

  // Update the error covariance matrix
  float tempP00 = error_covariance[axis][0][0];
  float tempP01 = error_covariance[axis][0][1];

  error_covariance[axis][0][0] -= kalmanGain[0] * tempP00;
  error_covariance[axis][0][1] -= kalmanGain[0] * tempP01;
  error_covariance[axis][1][0] -= kalmanGain[1] * tempP00;
  error_covariance[axis][1][1] -= kalmanGain[1] * tempP01;

  return predictedAngle;  // Return the updated angle
}

void updateKalmanFilter() {
  // Calculate the total accelerometer vector
  double totalAccelerometerVector = sqrt((accelerometerX * accelerometerX) + (accelerometerY * accelerometerY) + (accelerometerZ * accelerometerZ));
  
  // Calculate pitch and roll angles from accelerometer data
  pitch_angle_accelerometer[1] = asin(static_cast<float>(accelerometerX) / totalAccelerometerVector) * rad_to_deg;  // Pitch angle
  roll_angle_accelerometer[1] = asin(static_cast<float>(accelerometerY) / totalAccelerometerVector) * rad_to_deg;   // Roll angle

  // Update pitch and roll angles using the Kalman filter
  pitch_angle[1] = updateKalmanFilter(pitch_angle[1], pitch_angle_accelerometer[1], gyroscopeY, delta_time, 0);
  roll_angle[1] = updateKalmanFilter(roll_angle[1], roll_angle_accelerometer[1], gyroscopeX, delta_time, 1);
}
