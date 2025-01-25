# Attitude Estimation Algorithms

This project aims to test three different algorithms for estimating attitude: a Complementary Filter, a Kalman Filter, and a Madgwick Filter. The code is primarily designed for the ESP32 microcontroller but can be adapted for use with other microcontrollers as well.

## Algorithms

1. **Complementary Filter**
   - A simple and effective method for combining accelerometer and gyroscope data to estimate orientation.

2. **Kalman Filter**
   - A more advanced algorithm that uses a series of measurements observed over time to produce estimates that tend to be more precise than those based on a single measurement alone.

3. **Madgwick Filter**
   - An orientation filter that estimates the orientation of a sensor using a quaternion representation, providing robust performance in dynamic environments.

## Features

- Implements three different attitude estimation algorithms.
- Compares maximum run times of each algorithm.
- Configurable for various microcontrollers, with ESP32 as the primary target.

## Requirements

- ESP32 (or any compatible microcontroller)
- MPU6050
- Arduino IDE or PlatformIO

## Usage

1. Open the project in Arduino IDE or PlatformIO.

2. Upload the code to your microcontroller.

3. Monitor the output to compare the performance of each algorithm.

## Performance Results

|      Algorithms     |   0º   |  Max  |
|---------------------|--------|-------|
| ComplementaryFilter | 100ms  | 179ms |
| KalmanFilter        | 120ms  | 172ms |
| MadgwickFilter      | 166ms  | 242ms |

