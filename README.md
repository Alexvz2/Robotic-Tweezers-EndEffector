# Robotic-Tweezers-EndEffector

## Background
Capstone team 15 end-effector repository.
Capstone project has as an objective to create a force feedback mechanical arm with an enf-effector with the ability to sense and carry small insects and object without deforming them.

[Link to Arm Repo](https://github.com/JakeCronin1997/robot-tweezers)

## Folder Structure
- CAD - Solidworks 3D Model
- MC - Microcontroller Code
- COMMS - Communications Code
- MISC - Miscelenous

## Branches
Branches split between individual features, then merged w main.
- main
- motorPosition
- multi-scale
- remote control
- controller


## Todo
- [x] Printed 3d Prototype
- [x] Hardware Wiring and power for Demo
- [x] Pressure Units
    - [x] Load Cell Readings (HX711)
    - [x] Multi-scale Readings
    - [x] Noise elimination
- [x] Motor Controll
    - [x] PWM L298N Control
    - [x] PWM control + Rotation
    - [x] Hall Sensor Position Calculations
- [ ] Claw control
    - [x] Potientometer Rotation control
    - [x] Potientometer On/Off control
    - [ ] Pressure Serial Warning
    - [x] wifi phone monitor
    - [ ] wifi claw control
    - [ ] XBOX control
- [ ] Automation
    - [ ] Simple Loop Controller (Open and Close)
    - [ ] PID/LQR Controller

## Wiring Diagrams

## Components
- 1 L298N
- 2 HX711
- ESP32-12E
- 2 Load-cells 100g
- 6v 125 rpm motor w BLM
- 1 potientometer

## Libraries Used
- [HX711.h](https://github.com/bogde/HX711)
- Arduino.h
- [HX711-multi](https://github.com/compugician/HX711-multi)
