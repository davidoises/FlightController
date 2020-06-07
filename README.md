# EW ESP32 Flight Controller

This project is an **ESP32** based **Flight Controller**. This flight controller is designed to work with any quadrotor in X configuration, requiring only PID gains tuning.

# The Drone
Our drone is a quadrotor using **F450 airframe** with **A2212-13T 1000Kv** brushless motors and **1045 props**.


# The codes
This folder contains 3 versions that work for this configuration. Each version has different approaches to PID controllers mixing and an RTOS based code.
- FlightController-RTOS-Simple - RTOS based with simple (linear) PWM-to-Thrust aproximation
- FlightController-YPR-Simple  - No-RTOS simple (linear) PWM-to-Thrust aproximation
- FlightController-YPR-Real  - No-RTOS and more accurate PWM-to-Thrust aproximation (quadratic)

# Components

For basic control of attitude and altitude the flight controller operates with a **6DOF IMU** and a **barometer**. The board used in this design uses **BMX055** and **MS5611**.

A **LiPo** battery is required for BLDC and ESC power. This was tested on a **3S 5200mAh** battery, but is able to support **2S** batteries.

## User Interface

The board has several connectivity options, including **WiFi**, **BLE** and **950MHz** rf protocol.
For WiFi and BLE, **Blynk** was used as a phone interface to control altitude and orientation.
For 950MHz rf protocol, a **custom remote control** was designed.
