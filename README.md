# Arduino IAC Controller for Bosch Monopoint with Speeduino

This project uses an Arduino to control the Idle Air Control (IAC) valve for the Bosch Monopoint injection system. The Speeduino ECU will send signals to the Arduino, which will act as the controller for the IAC valve.

## Why This Project?
The Speeduino ECU supports three types of idle control:
1. On/Off
2. PWM/Duty Cycle for normal IAC valves
3. Stepper Motor

However, the Bosch Monopoint system from the 80s and 90s uses a unique setup with part numbers 0132008600 (DC motor) and 3437022 (potentiometer). This system works more like a throttle actuator, opening the butterfly valve enough for idle, similar to how a choke works in a carburetor. The injector is positioned before the butterfly valve, making it an electronic copy of a carburetor.

To control the Bosch Monopoint system with Speeduino, we need to convert the PWM/Duty Cycle signals from Speeduino into control signals for the butterfly valve. The 0132008600 part contains a DC motor, and the 3437022 part contains a potentiometer to measure the travel of the DC motor/butterfly valve. By combining these parts with an Arduino and the Speeduino signal, we can control the idle of the car.

## Components
- **Arduino**: Microcontroller used to control the IAC valve.
- **Speeduino**: Engine Control Unit (ECU) that sends signals to the Arduino.
- **Bosch Monopoint Injection System**: Fuel injection system that includes the IAC valve.
- **Idle Air Control (IAC) Valve**: Device that regulates the engine's idle speed by controlling the amount of air bypassing the throttle plate.
- **DC Motor**: Bosch part number 0132008600.
- **Potentiometer**: Bosch part number 3437022.

## Overview
The Speeduino ECU will send PWM (Pulse Width Modulation) signals to the Arduino. The Arduino will interpret these signals and adjust the IAC valve accordingly to maintain the desired idle speed.

## Prerequisites
- **Arduino IDE**: Ensure you have the Arduino IDE installed on your computer. You can download it from the [official website](https://www.arduino.cc/en/software).
- **Basic Electronics Knowledge**: Familiarity with basic electronics and wiring.
- **Components**: Ensure you have all the components listed in the "Components" section.

## Wiring Diagram
<!-- Wiring diagram image reference removed -->

- **Potentiometer**:
  - Connect one end to 5V on the Arduino.
  - Connect the other end to GND on the Arduino.
  - Connect the middle pin to A0 on the Arduino.

- **L293D Motor Driver**:
  - **Motor Pin 1**: Connect to pin 8 on the Arduino.
  - **Motor Pin 2**: Connect to pin 7 on the Arduino.
  - **Enable Pin**: Connect to pin 9 on the Arduino.
  - **Vcc1**: Connect to 5V on the Arduino.
  - **Vcc2**: Connect to an external power supply (e.g., 12V) for the motor.
  - **GND**: Connect to GND on the Arduino and the external power supply.

## Code
The Arduino code for this project can be found in the `IAC_Controller.ino` file. This file contains the logic to read the potentiometer value and control the DC motor to adjust the IAC valve position. You can also set the target position percentage via the serial monitor.

## Installation
1. **Install the Arduino IDE**: Download and install the Arduino IDE from the [official website](https://www.arduino.cc/en/software).
2. **Connect the Arduino**: Connect your Arduino board to your computer using a USB cable.
3. **Open the Code**: Open the provided Arduino code in the Arduino IDE.
4. **Select the Board and Port**: In the Arduino IDE, go to `Tools > Board` and select your Arduino board model. Then go to `Tools > Port` and select the port to which your Arduino is connected.
5. **Upload the Code**: Click the upload button in the Arduino IDE to upload the code to your Arduino board.

## Usage
1. **Upload the Code**: Upload the provided Arduino code to your Arduino board using the Arduino IDE.
2. **Connect the Components**: Follow the wiring diagram to connect the potentiometer, L293D motor driver, and IAC valve to the Arduino.
3. **Power the System**: Connect the Arduino to a power source and ensure the external power supply for the motor driver is also connected.
4. **Start the Engine**: Start your engine and let the Speeduino ECU send signals to the Arduino.
5. **Set Target Position**: Open the serial monitor in the Arduino IDE and enter the target position percentage (0-100) to set the desired idle speed.
6. **Monitor and Adjust**: The Arduino will automatically adjust the IAC valve based on the signals received from the Speeduino ECU to maintain the desired idle speed.

## Contributing
Contributions are welcome! If you have any suggestions or improvements, please create a pull request or open an issue on the project's GitHub repository.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
