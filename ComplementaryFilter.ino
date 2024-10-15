void initializeComplementaryFilter() {
  pitch_angle[0] = initial_pitch_angle;
  roll_angle[0] = initial_roll_angle;

  // Calculate gyroscope period based on sampling frequency
  gyroscope_period = delta_time;

  // Precompute yaw correction factor (gyro period * radians per degree)
  yaw_correction_factor = gyroscope_period * deg_to_rad;
}

void updateComplementaryFilter() {
  // Gyroscope angle calculations
  pitch_angle[0] -= gyroscopeY * delta_time;   // Update pitch angle using gyroscope Y-axis data
  roll_angle[0] += gyroscopeX * delta_time;    // Update roll angle using gyroscope X-axis data

  // Account for yaw-induced cross-coupling effects on pitch and roll
  pitch_angle[0] += roll_angle[0] * sin(gyroscopeZ * yaw_correction_factor);  // Adjust pitch for yaw
  roll_angle[0] += pitch_angle[0] * sin(gyroscopeZ * yaw_correction_factor);  // Adjust roll for yaw

  // Accelerometer angle calculations
  double total_acceleration = sqrt((accelerometerX * accelerometerX) + (accelerometerY * accelerometerY) + (accelerometerZ * accelerometerZ));  // Total accelerometer vector magnitude
  
  // Calculate pitch and roll angles using accelerometer data
  pitch_angle_accelerometer[0] = asin((float)accelerometerX / total_acceleration) * rad_to_deg;
  roll_angle_accelerometer[0] = asin((float)accelerometerY / total_acceleration) * rad_to_deg;

  // Complementary filter: combine gyroscope and accelerometer data to minimize drift
  pitch_angle[0] = pitch_angle[0] * 0.9996 + pitch_angle_accelerometer[0] * 0.0004;   // Correct pitch drift using accelerometer
  roll_angle[0] = roll_angle[0] * 0.9996 + roll_angle_accelerometer[0] * 0.0004;      // Correct roll drift using accelerometer
}
