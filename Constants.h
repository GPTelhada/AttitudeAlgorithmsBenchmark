#ifndef CONSTANTS_H
#define CONSTANTS_H

//Address
#define MPU6050_ADDRESS 0x68// I2C address of MPU6050 sensor
 
//Degrees and Radians
#define deg_to_rad 0.017453     // pi/180
#define rad_to_deg 57.295779    // 180/pi


//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
//#define ACCEL_2G //Default
#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


#define GYRO_FS_SEL_250    0x00 //B0000 0000
#define GYRO_FS_SEL_500    0x08 //B0000 1000
#define GYRO_FS_SEL_1000   0x10 //B0001 0000
#define GYRO_FS_SEL_2000   0x18 //B0001 1000

#define ACCEL_FS_SEL_2     0x00 //B0000 0000
#define ACCEL_FS_SEL_4     0x08 //B0000 1000
#define ACCEL_FS_SEL_8     0x10 //B0001 0000
#define ACCEL_FS_SEL_16    0x18 //B0001 1000


#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif


#endif