# Arduino Dash and Sensors for EV FSAE Car

## Overview
The **Arduino Dash** project for an FSAE electric vehicle (EV) involves integrating hardware and software to monitor and display key vehicle metrics such as speed, brake pressure, battery health, and cooling status. The system also supports **CAN** (Controller Area Network) communication for real-time data exchange, enhancing driver awareness and vehicle performance.

## Skills Utilized
- Arduino Programming
- C++ Development
- CAN Bus Communication

## Main Files

### 1. `ArduinoDash.ino`
This is the main Arduino sketch, responsible for initializing hardware, reading sensor data, processing that data, and handling CAN communication.

### 2. `can_helpers.hpp`
A helper file containing definitions, constants, and structures for CAN bus communication.

### 3. `helper_functions.hpp`
Contains additional helper functions for processing sensor data, updating visual indicators, and managing NeoPixel LEDs.

## File Breakdown

### ArduinoDash.ino
- **#Include Section**:
  - `can_helpers.hpp`: Custom CAN definitions and structures.
  - `Arduino_CAN.h`: CAN bus communication library.
  - `stdio.h`, `math.h`, `string.h`: Standard C libraries.
  - `Adafruit_NeoPixel.h`: NeoPixel LED control library.

- **Setup Function**:
  - Initializes serial and CAN communication.
  - Configures sensors and output pins.
  - Initializes NeoPixel LEDs.

- **Loop Function**:
  - Reads and processes sensor data.
  - Updates the battery health indicator.
  - Manages brake pressure, wheel speed, and steering angle data.
  - Handles CAN message transmission and reception.

### can_helpers.hpp
- Defines message IDs for CAN communication (e.g., `DRIVE_STATE_ID`, `VCU_STATES_ID`, etc.).
- Structures for CAN data transmission and reception.

### helper_functions.hpp
- Defines constants for LEDs, ADC values, and pin assignments.
- Functions to update LEDs based on vehicle state (e.g., battery health, fault indicators).

## Purpose of Arduino Dash
The Arduino Dash monitors and displays real-time data to ensure safe operation and performance of the EV FSAE car. It communicates with the Vehicle Control Unit (VCU), Accumulator Management System (AMS), and other subsystems using CAN, providing essential information like:
- Speed and brake status.
- Battery health.
- Cooling system status.
- Fault alerts.

## Hardware Components
- **Microcontroller (Arduino)**: Core processing unit for sensor readings and CAN communication.
- **LED Indicators**: Visual feedback for battery state, faults, and cooling system.
- **Buzzer**: Audio alerts for critical events, like "Ready to Drive."
- **CAN Bus Module**: Communicates with other vehicle subsystems.
- **Sensors**: Monitors brake pressure, wheel speed, steering angle, and battery health.

## Communication Systems
- **CAN Bus**: Exchanging data with VCU, AMS, and other subsystems.
- **Serial Communication**: Debugging during development.

## Software Features
- Reads and processes data from sensors.
- Updates LED and buzzer indicators.
- Transmits data over CAN bus.
- Monitors and reacts to vehicle states and faults.

## Indicators and Warnings
- **LED Indicators**: For state of charge, faults, cooling status, and AMS alerts.
- **Buzzer**: Plays melodies for different statuses (e.g., "Ready to Drive").

## Compliance with EV FSAE Rules
- **AMS Indicator Light**: Required red light for accumulator-related faults.
- **Dashboard Messaging**: Conforms to FSAE safety regulations for battery and fault monitoring.

## User Interface
The dashboard provides real-time data such as:
- Speed
- Brake pressure
- Battery health
- System faults

## Conclusion
The Arduino Dash for an EV FSAE car is a critical interface that enhances driver safety and vehicle performance through real-time monitoring and alerts. Its integration with CAN bus and use of sensors ensures continuous, accurate feedback to the driver.
