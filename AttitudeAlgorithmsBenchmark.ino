#include <Wire.h>
#include "Constants.h"

//Result
/*                       Sitting(0º)    Max
ComplementaryFilter  ->  100ms      179ms
KalmanFilter         ->  120ms      172ms
MadgwickFilter       ->  166ms      242ms

readMPUData() takes a max 603us to fetch the data at 400KHz

*/

// AXIS
// MPU6050 - X Front // Y left

// MPU
float accelerometerX, accelerometerY, accelerometerZ;
float previous_accelerometerX, previous_accelerometerY, previous_accelerometerZ;
float gyroscopeX, gyroscopeY, gyroscopeZ;
float previous_gyroscopeX, previous_gyroscopeY, previous_gyroscopeZ;
float yaw_correction_factor;
float temperature;

// Calculated Angles  CF - 0  KF - 1  MF - 2
float pitch_angle[3], roll_angle[3];
float pitch_angle_accelerometer[3], roll_angle_accelerometer[3];
float initial_pitch_angle, initial_roll_angle;

// Complementary Filter
double gyroscope_period, complementary_filter_helper;

// Kalman Filter
float estimated_angleX, estimated_angleY, estimated_angleZ;  // Estimated angles
float gyroscope_biasX, gyroscope_biasY, gyroscope_biasZ;     // Gyro bias
float error_covariance[2][2][2];              // Error covariance matrix
float kalman_Q_angle = 0.002;
float kalman_Q_bias = 0.004;
float kalman_R_measure = 0.04;

// Madgwick Filter
float madgwick_parameter_B = 0.04;   // Madgwick filter parameter
float quaternion_0 = 1.0f;
float quaternion_1 = 0.0f;
float quaternion_2 = 0.0f;
float quaternion_3 = 0.0f;

// Filter parameters
float low_pass_filter_accelerometer = 0.14;  // Accelerometer LP filter parameter (MPU6050 default: 0.14, MPU9250 default: 0.2)
float low_pass_filter_gyroscope = 0.1;      // Gyro LP filter parameter (MPU6050 default: 0.1, MPU9250 default: 0.17)
float low_pass_filter_magnetometer = 1.0;   // Magnetometer LP filter parameter

// MPU calibration parameters
float accelerometer_errorX = 0;
float accelerometer_errorY = 0;
float accelerometer_errorZ = 0;
float gyroscope_errorX = 1.10;
float gyroscope_errorY = -0.05;
float gyroscope_errorZ = 1.22;

// General timing variables
float delta_time;
unsigned long current_time_microseconds, previous_time_microseconds;
double loop_frequency = 400.0;
int print_counter = 0;

// Timing metrics
uint32_t timer_start = 0;
uint32_t execution_time[3];
uint32_t max_execution_time[3];
uint32_t data_fetching_time;
uint32_t max_data_fetching_time;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(1000);
  Serial.println("Start");

  initializeMPU();
  initializeComplementaryFilter();
  initializeKalmanFilter();
  delay(2000);
}

void loop() {
  previous_time_microseconds = current_time_microseconds;
  current_time_microseconds = micros();
  delta_time = (current_time_microseconds - previous_time_microseconds) / 1000000.0;

  start();
  readMPUData();
  data_fetching_time = stop();
  if (data_fetching_time > max_data_fetching_time) {
    max_data_fetching_time = data_fetching_time;
  }

  start();
  updateComplementaryFilter();
  execution_time[0] = stop();
  if (execution_time[0] > max_execution_time[0]) {
    max_execution_time[0] = execution_time[0];
  }

  start();
  updateKalmanFilter();
  execution_time[1] = stop();
  if (execution_time[1] > max_execution_time[1]) {
    max_execution_time[1] = execution_time[1];
  }

  start();
  updateMadgwick6DOF();

  execution_time[2] = stop();
  if (execution_time[2] > max_execution_time[2]) {
    max_execution_time[2] = execution_time[2];
  }

  // Printing Section
  if (print_counter == 20) {
    // printRollPitch();
    // Serial.print(",");
    printMaxExecutionTimes();
    Serial.println("");
    print_counter = 0;
  }
  print_counter++;

  regulateLoopRate(loop_frequency);
}
