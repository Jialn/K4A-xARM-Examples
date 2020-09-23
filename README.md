# xARM5 Gripping Example 
Example for xARM5 Gripping by Azure Kinect

## Overview
xARM5 Gripping Example

## Caution
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Please make sure the arm don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Installation
- Install AzureKinect SDK

  https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md

  There is no need to compile, just install the msi file

- Install xArm-Python-SDK

  ```
  git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
  cd xArm-Python-SDK
  pip install -e .
  ```

- Install other dependency

  pip install opencv-python pynput pywin32

## Usage
- Test Kinect marker detector

  python .\k4a_marker_det.py
  
- Contorl the arm using Keyboard/mouse or marker pos detected by Kinect

  python .\contoller_3d.py

- Run the gripping demostration
   
  python .\gripping_demo_3d_perception.py