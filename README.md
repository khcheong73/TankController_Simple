# Bluetooth Gamepad Controlled Tank Controller

## Overview
This project uses an ESP32 MINI and a DRV8835 motor driver to create a simple tank controller that can be controlled via a PS4 gamepad. It is suitable for remote-controlled vehicles and robotics applications.

## Hardware Requirements
- ESP32 MINI
- DRV8835 dual motor driver
- PS4 gamepad (or compatible)
- 2 DC motors (for tracks)
- Power source (battery)
- Various LEDs and resistors

## Wiring Diagram
https://github.com/khcheong73/TankController_Simple/blob/main/Schematic_RC-Tank-ESP32-MINIKIT-V1.1_2024-10-19.pdf

## Software Requirements
- Arduino IDE
- Bluepad32 library (https://github.com/ricardoquesada/bluepad32)

## Setup Instructions
1. Install the Arduino IDE.
2. Install the Bluepad32 library.
3. Upload the provided code to the ESP32.

## Usage Instructions
1. Power on the system.
2. Pair the PS4 gamepad with the ESP32 (Refer to Bluepad32 page)
3. Use the controls as described below.

## Controls
- **Left Analog Stick (up/down)**: Controls the left track.
- **Right Analog Stick (up/down)**: Controls the right track.
- **R2**: Accelerator.
- **Button O**: Toggle headlight.
- **Button X**: Toggle auxiliary light 1.
- **Button □**: Toggle auxiliary light 2.
- **Button △**: Toggle emergency light (blinks both left/right turn signals).

## Additional Features
- **Brake Lights**: Activate when the accelerator is not pressed.
- **Backward Light**: Automatically turns on when both left and right analog sticks are pushed down (indicating reverse).
- **Left/Right Turn Signal Lights**: Operate based on the analog stick controls for turning.
- **LED Indicators**: Provide visual feedback for various states of the vehicle (e.g., headlight, auxiliary lights).

## License
This code is provided for educational purposes. You are free to modify and use it as long as it is not for commercial purposes. Please give appropriate credit when using or modifying the code.

## Contributions
Feel free to fork the repository and submit pull requests for improvements!
