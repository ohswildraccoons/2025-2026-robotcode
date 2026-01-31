# 2026 Wild Raccoons FRC Code - FIRST AGE - Rebuilt
This is the source code for the 2026 robot from Oceanside High School's First Robotics Club (the Wild Raccoons).

# Getting Started

## Install Tooling
- Install WPILib tools: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

## Build Code
- Run the vscode task `WPILib: Build Robot Code`: Press Ctrl+Shift+P and type Build. (Cmd+Shift+P on MacOS)

## Simulate Code
- Run the vscode task `WPILib: Simulate Robot Code`: Press Ctrl+Shift+P and type Simulate. (Cmd+Shift+P on MacOS)
- Ensure keyboard 0 is attached to joystick 0. (This represents the driver controller)
- Ensure keyboard 1 is attached to joystick 1. (This represents the mechanism controller)
- Change the robot state to Teleoperated and use the below key mappings to drive the robot and move the mechanisms
- Key Mappings

| XBOX Joystick      | Simulator Name  | Keyboard 0 | Keyboard 1 |
|--------------------|-----------------|------------|------------|
| Left Stick Right   | Axis 0 Increase | d          |            |
| Left Stick Left    | Axis 0 Decrease | a          |            |
| Left Stick Up      | Axis 1 Increase | s          |            |
| Left Stick Down    | Axis 1 Decrease | w          |            |
| Right Stick Right  | Axis 2 Increase | t          |            |
| Right Stick Left   | Axis 2 Decrease | g          |            |
| Right Stick Up     | Axis 3 Decrease | h          |            |
| Right Stick Down   | Axis 3 Decrease | f          |            |
| A                  | Button 1        | z          |            |
| B                  | Button 2        | x          |            |
| X                  | Button 3        | c          |            |
| Y                  | Button 4        | v          |            |
| LB                 | Button 5        | b          |            |
| RB                 | Button 6        | n          |            |
| back               | Button 7        | m          |            |
| start              | Button 8        | ,          |            |
| Left Stick Button  | Button 9        | q          |            |
| Right Stick Button | Button 10       | r          |            |
| LT                 | Button 11       | e          |            |
| RT                 | Button 12       | y          |            |
| POV R              | POV 0 deg       | 8          |            |
| POV U R            | POV 45 deg      | 9          |            |
| POV U              | POV 90 deg      | 6          |            |
| POV U L            | POV 135 deg     | 3          |            |
| POV L              | POV 180 deg     | 2          |            |
| POV D L            | POV 225 deg     | 1          |            |
| POV D              | POV 270 deg     | 4          |            |
| POV D R            | POV 315 deg     | 7          |            |

## Deploy Code
- Attach the computer to the RoboRio via ethernet cable. You may need a USB-C to ethernet adapter.
- Run the vscode task `WPILib: Deploy Robot Code`: Press Ctrl+Shift+P and type Deploy. (Cmd+Shift+P on MacOS)

## Drive Robot
- Open the driver's station (Only works on Windows machines)