# IAC Servo Controller for Bosch Monopoint System

This Arduino project implements a servo controller for the Bosch 0132008600 throttle actuator, using Speeduino's PWM idle output as input and Track 1 of the Bosch 3437022 TPS for position feedback.

## System Overview

The Arduino acts as a dedicated servo driver, translating PWM duty cycle commands from Speeduino into precise throttle position control for idle air control. All idle control logic (open loop, closed loop, target calculations) remains in Speeduino/TunerStudio - this controller only provides the servo functionality.

## Hardware Requirements

### Essential Components
- **Arduino Uno/Nano** - Main controller
- **Bosch 0132008600** - DC motor throttle actuator
- **Bosch 3437022 TPS** - Dual-track throttle position sensor
- **H-Bridge Motor Driver** - L298N or similar (5A+ capability)
- **Power Supply** - 12V for motor, 5V for Arduino

### Wiring Diagram (1 PWM Mode)

```
Speeduino PWM (Open Drain) -----> Pin 2 (Arduino with internal pull-up enabled)

Bosch 3437022 TPS (5-pin connector):
Pin 5: +5V -----------> 5V Supply
Pin 4: Track 2 -------> Speeduino TPS Input [Main TPS Signal]
Pin 3: (Not used) ----> (Not connected)
Pin 2: Track 1 -------> Pin A0 (Arduino) [Servo Feedback]
Pin 1: Ground --------> Common Ground

Bosch 0132008600 Motor (4-pin connector):
Pin 1: Motor + ----------> H-Bridge Output A
Pin 2: Motor - ----------> H-Bridge Output B  
Pin 3: CTS Idle Switch -> (Not used)
Pin 4: CTS Idle Switch -> (Not used)

Arduino Pin 5 --------> H-Bridge PWM Input
Arduino Pin 6 --------> H-Bridge Direction Input

12V Supply -----------> H-Bridge VCC, Motor Supply
5V Supply ------------> Arduino VCC, TPS Pin 5
Ground ---------------> Common Ground (All components)
```

## Pin Configuration

| Arduino Pin | Function | Connection |
|-------------|----------|------------|
| Pin 2 | PWM Input | Speeduino Idle Output (Arduino internal pull-up enabled) |
| Pin A0 | TPS Feedback | TPS Pin 2 (Track 1 - 0-5V) |
| Pin 5 | Motor PWM | H-Bridge PWM Input |
| Pin 6 | Motor Direction | H-Bridge Direction |
| Pin 13 | Status LED | Built-in LED |

## Hardware Component Details

### Bosch 3437022 TPS Pinout (5-pin connector)
- **Pin 1**: Ground (GND)
- **Pin 2**: Track 1 Output (0-5V servo feedback) - Connect to Arduino A0
- **Pin 3**: (Not used)  
- **Pin 4**: Track 2 Output (0-5V main TPS signal) - Connect to Speeduino TPS input
- **Pin 5**: +5V Supply (Power)

*Note: This is a 5-pin connector with dual potentiometers. Pin 3 is not used in this configuration.*

### Bosch 0132008600 Motor Specifications - DETAILED PINOUT
- **Type**: DC Motor Actuator with 4-pin connector
- **Voltage**: 12V nominal (9-14V operating range)
- **Current**: ~2-5A peak (varies with load)
- **Connector**: 4-pin electrical connector
- **Function**: Controls throttle butterfly resting position (idle air bypass)

#### Bosch 0132008600 Pinout (4-pin connector):
- **Pin 1**: DC Motor + (Positive terminal)
- **Pin 2**: DC Motor - (Negative terminal)  
- **Pin 3**: CTS Idle Switch (Closed Throttle Switch - idle position detection)
- **Pin 4**: CTS Idle Switch (Closed Throttle Switch - idle position detection)

*Note: Pins 1 & 2 are the actual DC motor connections. Pins 3 & 4 are closed throttle switch contacts used by some ECUs to detect when throttle is at idle position.*

#### H-Bridge Connection:
- **H-Bridge Output A** → Pin 1 (Motor positive)
- **H-Bridge Output B** → Pin 2 (Motor negative)
- **Arduino Pin 5** → H-Bridge PWM Input (speed control)
- **Arduino Pin 6** → H-Bridge Direction Input (polarity control)
- **Pins 3 & 4**: Not used in this application (CTS function handled by TPS Track 1)

#### Alternative Part Numbers:
- BMW: 13541464908
- VW/Audi: 051133031 / 51133031  
- Peugeot/Citroën: 1920F8, 95651031
- Fiat: 7077494
- Applications: 1988-1999 Audi, BMW, Citroën, Fiat, Lancia, Peugeot, Renault, Seat, Škoda, VW

#### Technical Notes:
- **Operating Mode**: Electric reversible actuator
- **Control Method**: PWM speed control + direction switching via H-Bridge
- **Mounting**: Integrated into throttle body assembly
- **Thread/Shaft**: Mechanical connection to throttle butterfly valve
- **Duty Cycle**: Typical 0-100% PWM for position control

### Speeduino PWM Output - CRITICAL WIRING NOTE
- **Type**: Open Drain Output (NOT push-pull)
- **Arduino Solution**: Use **internal pull-up resistor** (enabled in software)
- **No external resistor needed**: Arduino's internal ~20-50kΩ pull-up handles this
- **Wiring**: Direct connection: Speeduino PWM → Arduino Pin 2
- **Signal Level**: 0V (when Speeduino pulls low) to 5V (when pulled high by internal resistor)

## Speeduino/TunerStudio Configuration

### Required Settings in TunerStudio:

1. **Enable Idle Control:**
   - Go to Settings → Idle
   - Enable "Use Idle Control"
   - Set Algorithm to "PWM Open Loop" or "PWM Closed Loop"

2. **Configure PWM Output:**
   - **PWM Type**: Choose between "1 PWM" or "2 PWM" in TunerStudio Idle settings
   - **Recommended**: Use "1 PWM" mode for this Arduino controller
   - Set PWM Frequency: Use available Speeduino options (typically 15.6Hz, 31.25Hz, 62.5Hz, 125Hz, or 250Hz)
   - **Note**: Arduino code automatically adapts to any frequency - no specific frequency required
   - Set Duty Cycle range: 10-90% (maps to 1000-2000μs pulse width)
   - Assign Idle PWM output to appropriate pin
   - **Important**: Speeduino PWM is open drain - Arduino internal pull-up handles this automatically

3. **TPS Configuration:**
   - Wire TPS Pin 5 to +5V supply
   - Wire TPS Pin 2 (Track 1) to Arduino A0 for servo feedback
   - Wire TPS Pin 1 to ground  
   - Wire TPS Pin 4 (Track 2) to Speeduino TPS input for engine management
   - Calibrate TPS in TunerStudio (closed/open throttle positions)
   - Track 1 and Track 2 provide independent position signals

### PWM Signal Types Explained:

**1 PWM Mode (Recommended for this project):**
- Single PWM signal with variable duty cycle (10-90%)
- Duty cycle represents desired position: 50% = center, 90% = full open, 10% = full closed
- Arduino reads duty cycle and determines both direction and magnitude
- **Connection**: Single wire from Speeduino PWM output → Arduino Pin 2

**2 PWM Mode (Alternative - requires code modification):**
- Two separate PWM signals: one for each direction (open/close)
- Each signal can be 0-100% duty cycle
- Typically used for dual-coil stepper motors or H-bridge direct control
- **Connection**: Would require two Arduino input pins and modified code
- **Note**: Current Arduino code is designed for 1 PWM mode only

**Why 1 PWM is Recommended:**
- Simpler wiring (single signal wire)
- Current Arduino code is optimized for this mode
- Compatible with most Speeduino idle control algorithms
- Easier troubleshooting and setup

## Software Features

### Core Functionality
- **PWM Input Reading**: Measures 1000-2000μs pulse width from Speeduino (1 PWM mode)
- **Position Feedback**: Reads TPS Track 1 for current throttle position
- **PID Control**: Smooth, accurate position control with tunable parameters
- **Motor Drive**: Bidirectional PWM control via H-Bridge
- **Direction Logic**: Automatically determines motor direction from duty cycle

### Safety Features
- Input validation and filtering
- Motor enable/disable capability
- Position limits and deadband
- Integral windup protection

### Diagnostic Features
- Serial monitor output (115200 baud)
- Real-time parameter tuning via serial commands
- Status LED indication
- Comprehensive error reporting

## Serial Commands

Connect to Arduino via Serial Monitor (115200 baud) for diagnostics and tuning:

| Command | Description |
|---------|-------------|
| `STATUS` | Show detailed system status |
| `KP=value` | Set proportional gain (e.g., KP=2.0) |
| `KI=value` | Set integral gain (e.g., KI=0.5) |
| `KD=value` | Set derivative gain (e.g., KD=0.1) |
| `ENABLE` | Enable motor control |
| `DISABLE` | Disable motor control |
| `HELP` | Show all available commands |

## Installation and Setup

1. **Hardware Assembly:**
   - Wire components according to the diagram above
   - **Note**: No external pull-up resistor needed - Arduino handles this internally
   - Ensure proper power supply (12V for motor, 5V for Arduino)
   - Double-check all connections before powering on

2. **Software Upload:**
   - Open `IAC_Servo_Controller.ino` in Arduino IDE
   - Select correct board (Arduino Uno/Nano)
   - Upload code to Arduino

3. **Initial Testing:**
   - Open Serial Monitor (115200 baud)
   - Type `STATUS` to verify system startup
   - Manually test motor with `ENABLE`/`DISABLE` commands

4. **PID Tuning:**
   - Start with default values (Kp=2.0, Ki=0.5, Kd=0.1)
   - Adjust via serial commands while monitoring response
   - Tune for smooth, stable operation without oscillation

## Integration with Speeduino

1. Configure TunerStudio idle settings as described above
2. Connect PWM output from Speeduino to Arduino Pin 2
3. Wire TPS Pin 4 (Track 2) to Speeduino TPS input for engine management
4. Wire TPS Pin 2 (Track 1) to Arduino A0 for servo feedback
5. Ensure proper power supply connections (12V motor, 5V Arduino/TPS)
6. Test idle control operation in TunerStudio

## Troubleshooting

### Common Issues:
- **No PWM detected**: Check Speeduino idle output configuration and wiring
- **Erratic movement**: Verify power supply stability and H-Bridge connections
- **Poor tracking**: Adjust PID parameters via serial commands
- **Motor not responding**: Check H-Bridge wiring and motor connections

### Diagnostic Steps:
1. Verify all power connections (12V motor, 5V Arduino/TPS, Ground)
2. Check PWM signal with oscilloscope or serial monitor (should be 0-5V)
3. Confirm TPS Track 1 voltage changes (0-5V) with throttle movement  
4. Verify TPS Track 2 signal reaches Speeduino correctly
5. Test motor manually with `ENABLE`/`DISABLE` commands
6. Check H-Bridge connections and motor polarity

## Technical Specifications

- **Control Loop Frequency**: 100Hz (10ms intervals)
- **PWM Input Mode**: 1 PWM (single signal with variable duty cycle)
- **PWM Input Range**: 1000-2000μs (500-2500μs accepted)
- **Speeduino PWM Frequency**: 15.6Hz to 250Hz (configurable in TunerStudio)
- **Arduino PWM Output**: ~490Hz (to H-Bridge motor driver)
- **TPS Resolution**: 10-bit (0-1023)
- **Serial Baud Rate**: 115200
- **Operating Voltage**: 5V (Arduino), 12V (Motor)

*Note: For 2 PWM mode (dual signals), code modification would be required to read two separate PWM inputs.*

## License

This project is open source. Use and modify as needed for your application.

## Support

For technical support or questions, refer to the Speeduino community forums or the project documentation.
