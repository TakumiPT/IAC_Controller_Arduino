# Arduino IAC Controller for Bosch Motronic with Speeduino

This project uses an Arduino to control the Idle Air Control (IAC) valve for the Bosch Motronic injection system. The Speeduino ECU sends signals to the Arduino, which acts as the controller for the IAC valve.

![Bosch Motronic](https://upload.wikimedia.org/wikipedia/commons/thumb/f/f4/Bosch_monopoint.jpg/2560px-Bosch_monopoint.jpg)

## Why This Project?
The Speeduino ECU supports three types of idle control:
1. On/Off
2. PWM/Duty Cycle for normal IAC valves
3. Stepper Motor

However, the Bosch Motronic system from the 80s and 90s uses a unique setup with part numbers 0132008600 (DC motor) and 3437022 (throttle position sensor). This system works like a throttle actuator, opening the butterfly valve enough for idle, similar to a carburetor choke. The injector is positioned before the butterfly valve, making it an electronic copy of a carburetor.

To control the Bosch Motronic system with Speeduino, we need to convert the PWM/Duty Cycle signals from Speeduino into control signals for the butterfly valve. The 0132008600 part contains a DC motor, and the 3437022 part contains two potentiometers to measure the travel of the DC motor/butterfly valve and the accelerator travel. By combining these parts with an Arduino and the Speeduino signal, we can control the car's idle.

## Components
- **Arduino**: Microcontroller used to control the IAC valve.
- **Speeduino**: Engine Control Unit (ECU) that sends signals to the Arduino.
- **Bosch Motronic Injection System**: Fuel injection system that includes the IAC valve.
- **Idle Air Control (IAC) Valve**: Device that regulates the engine's idle speed by controlling the amount of air bypassing the throttle plate.
- **DC Motor**: Bosch part number 0132008600.
  ![Bosch 0132008600](https://www.autotav.com/bilder/produkte/gross/Leerlaufsteller-Bosch-0132008600-Drosselklappensteller.jpg)
  - **Pin 1**: DC motor pin
  - **Pin 2**: DC motor pin
  - **Pin 3**: CTS idle button (used by some ECUs to detect when the accelerator is released and comes to resting place)
  - **Pin 4**: CTS idle button (used by some ECUs to detect when the accelerator is released and comes to resting place)
- **Throttle Position Sensor (TPS)**: Bosch part number 3437022, contains two potentiometers.
  ![Bosch 3437022](https://2.bp.blogspot.com/-KULqzjvxB94/V3sY4aP58WI/AAAAAAAAAH0/Py8Fm3HYiD4Ffd6RAuULeemBGt2gdXdXwCLcB/s1600/THROTTLE-POSITION-SENSOR-FOR-VW-037907385A-3437022%2B%25284%2529.jpg)
  - **Pin 1**: GND
  - **Pin 2**: Signal for Arduino (IAC travel)
  - **Pin 3**: (does not exist)
  - **Pin 4**: Signal for TPS of Speeduino (accelerator pedal positions/travel)
  - **Pin 5**: Power 5V
- **L293D Motor Driver**: Motor driver IC used to control the direction and speed of the DC motor.
  ![L293D Motor Driver](https://how2electronics.com/wp-content/uploads/2022/08/L293D-Motor-Driver-IC-Pin.jpg)

## Overview
The Speeduino ECU sends PWM (Pulse Width Modulation) signals to the Arduino. The Arduino interprets these signals and adjusts the IAC valve accordingly to maintain the desired idle speed.

## Prerequisites
- **Arduino IDE**: Ensure you have the Arduino IDE installed on your computer. You can download it from the [official website](https://www.arduino.cc/en/software).
- **Basic Electronics Knowledge**: Familiarity with basic electronics and wiring.
- **Components**: Ensure you have all the components listed in the "Components" section.

## Wiring Diagram
<!-- Wiring diagram image reference removed -->

- **Throttle Position Sensor (TPS)**:
  - **Pin 1**: Connect to GND on the Arduino.
  - **Pin 2**: Connect to A0 on the Arduino (IAC travel signal).
  - **Pin 5**: Connect to 5V on the Arduino.

- **L293D Motor Driver**:
  - **Input Pin 1**: Connect to pin 8 on the Arduino.
  - **Input Pin 2**: Connect to pin 7 on the Arduino.
  - **Enable Pin**: Connect to pin 9 on the Arduino.
  - **Vcc1**: Connect to 5V on the Arduino.
  - **Vcc2**: Connect to an external power supply (e.g., 12V) for the motor.
  - **GND**: Connect to GND on the Arduino and the external power supply.
  - **Output Pin 1**: Connect to pin 1 on the DC motor.
  - **Output Pin 2**: Connect to pin 2 on the DC motor.

### Why Use the L293D Motor Driver?
The L293D motor driver is used to control the direction and speed of the DC motor. Unlike simple positive and negative connections, the L293D allows for more precise control over the motor's operation. By using input pins to control the motor's direction, the L293D can reverse the motor's rotation, which is essential for adjusting the IAC valve position accurately.

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

## Next Phase: Reading Speeduino V4 Open-Drain Signal

The next phase of this project is to understand how to read the Speeduino V4 idle PWM open-drain signal and use it with the Arduino. This signal will control the target position value for the IAC valve.

### What is an Open-Drain Signal?
An open-drain (or open-collector) signal is a type of output that can either be connected to ground (low) or left floating (high-impedance). It requires an external pull-up resistor to pull the signal to a high voltage level when the output is not actively pulling it low.

### How to Connect an Open-Drain Signal to Arduino
To connect an open-drain signal from the Speeduino V4 to the Arduino, follow these steps:
1. **Pull-Up Resistor**: Connect a pull-up resistor (e.g., 10k ohms) between the open-drain signal line and the 5V supply on the Arduino.
2. **Signal Line**: Connect the open-drain signal line to one of the digital input pins on the Arduino.

### Reading an Open-Drain PWM Signal with Arduino
To read the open-drain PWM signal from the Speeduino V4 with the Arduino:
1. **Configure the Pin**: Set the digital input pin connected to the open-drain signal as an input in the Arduino code.
2. **Read the Signal**: Use the `pulseIn()` function to measure the duration of the high and low pulses of the PWM signal.

Example code snippet:
```cpp
const int pwmPin = 2; // Digital input pin connected to the open-drain signal
unsigned long highDuration;
unsigned long lowDuration;

void setup() {
  pinMode(pwmPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  highDuration = pulseIn(pwmPin, HIGH);
  lowDuration = pulseIn(pwmPin, LOW);
  Serial.print("High Duration: ");
  Serial.print(highDuration);
  Serial.print(" Low Duration: ");
  Serial.println(lowDuration);
  delay(1000);
}
```

This code will read the high and low durations of the PWM signal and print them to the serial monitor. The measured PWM signal will be used to control the target position value for the IAC valve.

## Additional Resources
- [Speeduino Official Website](https://speeduino.com/)
- [Speeduino Wiki](https://wiki.speeduino.com/)
- [Speeduino Idle Configuration (PWM Option)](https://wiki.speeduino.com/en/configuration/Idle)
- [Bosch Motronic on Wikipedia](https://en.wikipedia.org/wiki/Motronic)

## Contributing
Contributions are welcome! If you have any suggestions or improvements, please create a pull request or open an issue on the project's GitHub repository.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
