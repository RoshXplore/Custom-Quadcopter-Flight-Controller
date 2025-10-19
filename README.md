# STM32F407 Flight Controller

A custom quadcopter flight controller firmware for the **STM32F407** microcontroller, written in C using the **STM32CubeIDE** and **HAL** libraries.

This project implements a stable flight controller using an MPU6050 IMU and a cascade PID control system.

## 🚁 Core Features

* **Microcontroller:** STM32F407VGT6
* **IMU:** MPU6050 (Accelerometer + Gyroscope)
* **Stabilization:** Cascade PID controller (Angle Outer Loop, Rate Inner Loop) for Pitch and Roll.
* **Angle Estimation:** Kalman Filter for robust sensor fusion (combines gyro and accelerometer data).
* **RC Input:** 4-channel PWM input capture (for Roll, Pitch, Throttle, Yaw) compatible with receivers like the FlySky FS-CT6B.
* **Motor Output:** 4-channel PWM for brushless motors (ESCs).

## 🛠️ Hardware & Pinout

This firmware is configured for the following hardware setup. Pin assignments must match your STM32CubeIDE (`.ioc`) configuration.

* **IMU:** MPU6050 connected via **I2C2**
    * `PB10`: I2C2_SCL
    * `PB11`: I2C2_SDA
* **Receiver (PWM Input):**
    * `PA0` (TIM2_CH1): RC Channel 1 (Roll)
    * `PA5` (TIM3_CH1): RC Channel 2 (Pitch)
    * `PB0` (TIM4_CH1): RC Channel 3 (Throttle)
    * `PC6` (TIM12_CH1): RC Channel 4 (Yaw)
* **Motors (PWM Output):**
    * `PA10` (TIM1_CH3): Motor 1 (Front-Left)
    * `PA11` (TIM1_CH4): Motor 2 (Front-Right)
    * `PB14` (TIM9_CH1): Motor 3 (Back-Right)
    * `PB15` (TIM9_CH2): Motor 4 (Back-Left)

## 🧠 Control Algorithms

### 1. Sensor Fusion (Kalman Filter)
The code in `mpu6050.c` uses a Kalman filter to fuse data from the accelerometer and gyroscope.
* **Gyroscope:** Provides fast, short-term rotation data but drifts over time.
* **Accelerometer:** Provides slow, long-term gravity reference (level) but is noisy and affected by movement.
* **Result:** The filter combines both to produce a stable, responsive, and drift-free angle estimation (`KalmanAngleX` and `KalmanAngleY`).

### 2. Cascade PID Controller
The core stabilization logic in `PID_control.c` uses two nested PID loops:
* **Outer Loop (Angle):** Compares the desired angle (from the remote) to the current angle (from the Kalman filter). Its output is a *target rotational rate*.
* **Inner Loop (Rate):** Compares the target rate (from the outer loop) to the current rate (from the raw gyro). Its output is the final correction value sent to the motors. This allows the controller to react very quickly to sudden gusts or tumbles.

### 3. Motor Mixing
The final outputs from the PID controllers (for Pitch and Roll) are combined with the user's Throttle and Yaw commands using a standard "Quad-X" mixing algorithm to determine the final speed for each of the four motors.


### **⚠️ SAFETY WARNING ⚠️**

**ALWAYS REMOVE PROPELLERS** before flashing, testing, or debugging your quadcopter. Incorrect PID values or motor mixing can cause the drone to behave erratically.
