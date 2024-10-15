void updateMadgwick6DOF() {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2quaternion_0, _2quaternion_1, _2quaternion_2, _2quaternion_3, _4quaternion_0, _4quaternion_1, _4quaternion_2 ,_8quaternion_1, _8quaternion_2, quaternion_0quaternion_0, quaternion_1quaternion_1, quaternion_2quaternion_2, quaternion_3quaternion_3;

  //Convert gyroscopeYroscope degrees/sec to radians/sec
  gyroscopeX *= 0.0174533f;
  gyroscopeY *= 0.0174533f;
  gyroscopeZ *= 0.0174533f;

  //Rate of change of quaternion from gyroscopeYroscope
  qDot1 = 0.5f * (-quaternion_1 * gyroscopeX - quaternion_2 * gyroscopeY - quaternion_3 * gyroscopeZ);
  qDot2 = 0.5f * (quaternion_0 * gyroscopeX + quaternion_2 * gyroscopeZ - quaternion_3 * gyroscopeY);
  qDot3 = 0.5f * (quaternion_0 * gyroscopeY - quaternion_1 * gyroscopeZ + quaternion_3 * gyroscopeX);
  qDot4 = 0.5f * (quaternion_0 * gyroscopeZ + quaternion_1 * gyroscopeY - quaternion_2 * gyroscopeX);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((accelerometerX == 0.0f) && (accelerometerY == 0.0f) && (accelerometerZ == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = fastInverseSqrt(accelerometerX * accelerometerX + accelerometerY * accelerometerY + accelerometerZ * accelerometerZ);
    accelerometerX *= recipNorm;
    accelerometerY *= recipNorm;
    accelerometerZ *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2quaternion_0 = 2.0f * quaternion_0;
    _2quaternion_1 = 2.0f * quaternion_1;
    _2quaternion_2 = 2.0f * quaternion_2;
    _2quaternion_3 = 2.0f * quaternion_3;
    _4quaternion_0 = 4.0f * quaternion_0;
    _4quaternion_1 = 4.0f * quaternion_1;
    _4quaternion_2 = 4.0f * quaternion_2;
    _8quaternion_1 = 8.0f * quaternion_1;
    _8quaternion_2 = 8.0f * quaternion_2;
    quaternion_0quaternion_0 = quaternion_0 * quaternion_0;
    quaternion_1quaternion_1 = quaternion_1 * quaternion_1;
    quaternion_2quaternion_2 = quaternion_2 * quaternion_2;
    quaternion_3quaternion_3 = quaternion_3 * quaternion_3;

    //Gradient decent algorithm corrective step
    s0 = _4quaternion_0 * quaternion_2quaternion_2 + _2quaternion_2 * accelerometerX + _4quaternion_0 * quaternion_1quaternion_1 - _2quaternion_1 * accelerometerY;
    s1 = _4quaternion_1 * quaternion_3quaternion_3 - _2quaternion_3 * accelerometerX + 4.0f * quaternion_0quaternion_0 * quaternion_1 - _2quaternion_0 * accelerometerY - _4quaternion_1 + _8quaternion_1 * quaternion_1quaternion_1 + _8quaternion_1 * quaternion_2quaternion_2 + _4quaternion_1 * accelerometerZ;
    s2 = 4.0f * quaternion_0quaternion_0 * quaternion_2 + _2quaternion_0 * accelerometerX + _4quaternion_2 * quaternion_3quaternion_3 - _2quaternion_3 * accelerometerY - _4quaternion_2 + _8quaternion_2 * quaternion_1quaternion_1 + _8quaternion_2 * quaternion_2quaternion_2 + _4quaternion_2 * accelerometerZ;
    s3 = 4.0f * quaternion_1quaternion_1 * quaternion_3 - _2quaternion_1 * accelerometerX + 4.0f * quaternion_2quaternion_2 * quaternion_3 - _2quaternion_2 * accelerometerY;
    recipNorm = fastInverseSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= madgwick_parameter_B * s0;
    qDot2 -= madgwick_parameter_B * s1;
    qDot3 -= madgwick_parameter_B * s2;
    qDot4 -= madgwick_parameter_B * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  quaternion_0 += qDot1 * delta_time;
  quaternion_1 += qDot2 * delta_time;
  quaternion_2 += qDot3 * delta_time;
  quaternion_3 += qDot4 * delta_time;

  //Normalise quaternion
  recipNorm = fastInverseSqrt(quaternion_0 * quaternion_0 + quaternion_1 * quaternion_1 + quaternion_2 * quaternion_2 + quaternion_3 * quaternion_3);
  quaternion_0 *= recipNorm;
  quaternion_1 *= recipNorm;
  quaternion_2 *= recipNorm;
  quaternion_3 *= recipNorm;

  //Compute angles
  roll_angle[2] = atan2(quaternion_0*quaternion_1 + quaternion_2*quaternion_3, 0.5f - quaternion_1*quaternion_1 - quaternion_2*quaternion_2)*rad_to_deg; //degrees
  pitch_angle[2] = -asin(-2.0f * (quaternion_1*quaternion_3 - quaternion_0*quaternion_2))*rad_to_deg; //degrees
  //yaw_IMU = -atan2(quaternion_1*quaternion_2 + quaternion_0*quaternion_3, 0.5f - quaternion_2*quaternion_2 - quaternion_3*quaternion_3)*rad_to_deg; //degrees
}