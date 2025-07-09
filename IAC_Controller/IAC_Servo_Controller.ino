/*
 * IAC Servo Controller for Bosch Monopoint System
 * 
 * This Arduino acts as a servo controller for the Bosch 0132008600 throttle actuator.
 * It takes PWM input from Speeduino's idle output and uses Track 1 of the Bosch 3437022 TPS
 * for position feedback to achieve precise idle throttle control.
 * 
 * Hardware:
 * - Bosch 0132008600 DC motor actuator (4-pin: Motor+, Motor-, CTS, CTS)
 * - Bosch 3437022 TPS (5-pin: GND, Track1, unused, Track2, +5V)
 * - H-Bridge motor driver (e.g., L298N or similar, 5A+ capability)
 * - Arduino Uno/Nano
 * 
 * Wiring:
 * - Pin 2: PWM input from Speeduino idle output (OPEN DRAIN - internal pull-up enabled)
 * - Pin A0: TPS Pin 2 (Track 1) feedback (0-5V)
 * - Pins 5,6: H-Bridge motor control (PWM + Direction)
 * - Pin 13: Status LED
 * 
 * TPS Connections (3437022 5-pin):
 * - Pin 1: Ground
 * - Pin 2: Track 1 output (0-5V) -> Arduino A0 [Servo Feedback]
 * - Pin 3: Not used
 * - Pin 4: Track 2 output (0-5V) -> Speeduino TPS input [Main TPS Signal]
 * - Pin 5: +5V supply
 * 
 * Motor Connections (0132008600 4-pin):
 * - Pin 1: Motor + → H-Bridge Output A
 * - Pin 2: Motor - → H-Bridge Output B  
 * - Pin 3: CTS idle switch (not used)
 * - Pin 4: CTS idle switch (not used)
 * 
 * PWM Input Circuit:
 * Speeduino PWM (Open Drain) -----> Arduino Pin 2 (internal pull-up enabled)
 * 
 * Author: Arduino IAC Controller
 * Version: 1.0
 * Date: 2024
 */

// Pin definitions
const int PWM_INPUT_PIN = 2;        // PWM input from Speeduino (open drain, internal pull-up)
const int TPS_FEEDBACK_PIN = A0;    // TPS Pin 2 (Track 1) analog input
const int MOTOR_PWM_PIN = 5;        // Motor PWM output
const int MOTOR_DIR_PIN = 6;        // Motor direction output
const int STATUS_LED_PIN = 13;      // Status LED

// PID Controller parameters
float Kp = 2.0;    // Proportional gain
float Ki = 0.5;    // Integral gain  
float Kd = 0.1;    // Derivative gain

// PID variables
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// PWM measurement variables
volatile unsigned long pwmRiseTime = 0;
volatile unsigned long pwmPulseWidth = 0;
volatile bool newPwmData = false;

// Motor control variables
int targetPosition = 0;     // Target position (0-1023)
int currentPosition = 0;    // Current TPS feedback position
int motorOutput = 0;        // Motor PWM output (-255 to +255)

// Timing variables
unsigned long lastControlUpdate = 0;
unsigned long lastSerialOutput = 0;
const unsigned long CONTROL_INTERVAL = 10;  // 10ms control loop
const unsigned long SERIAL_INTERVAL = 100;  // 100ms serial output

// Safety and calibration
const int MIN_POSITION = 0;     // Minimum TPS position
const int MAX_POSITION = 1023;  // Maximum TPS position
const int DEADBAND = 5;         // Position deadband to prevent oscillation
bool motorEnabled = true;       // Motor enable flag

void setup() {
  Serial.begin(115200);
  
  // Configure pins
  pinMode(PWM_INPUT_PIN, INPUT_PULLUP);  // Enable internal pull-up for open drain PWM
  pinMode(TPS_FEEDBACK_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Attach interrupt for PWM measurement
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmInterrupt, CHANGE);
  
  // Initialize motor to stopped state
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  
  // Flash LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("IAC Servo Controller Started");
  Serial.println("PWM Input: Internal pull-up enabled for open drain signal");
  Serial.println("Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update control loop at fixed interval
  if (currentTime - lastControlUpdate >= CONTROL_INTERVAL) {
    lastControlUpdate = currentTime;
    
    // Read current position from TPS Track 1
    currentPosition = analogRead(TPS_FEEDBACK_PIN);
    
    // Update target position from PWM input
    updateTargetFromPWM();
    
    // Run PID control
    if (motorEnabled) {
      motorOutput = calculatePID(targetPosition, currentPosition);
      driveMotor(motorOutput);
    } else {
      analogWrite(MOTOR_PWM_PIN, 0);
    }
    
    // Update status LED (blink when active)
    digitalWrite(STATUS_LED_PIN, (currentTime / 500) % 2);
  }
  
  // Serial output for monitoring
  if (currentTime - lastSerialOutput >= SERIAL_INTERVAL) {
    lastSerialOutput = currentTime;
    printStatus();
  }
  
  // Check for serial commands
  handleSerialCommands();
}

// PWM interrupt handler
void pwmInterrupt() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    // Rising edge - start of pulse
    pwmRiseTime = currentTime;
  } else {
    // Falling edge - end of pulse
    if (pwmRiseTime > 0) {
      unsigned long pulseWidth = currentTime - pwmRiseTime;
      
      // Filter out noise (valid PWM should be 1000-2000us)
      if (pulseWidth >= 500 && pulseWidth <= 2500) {
        pwmPulseWidth = pulseWidth;
        newPwmData = true;
      }
    }
  }
}

// Update target position based on PWM input
void updateTargetFromPWM() {
  if (newPwmData) {
    newPwmData = false;
    
    // Convert PWM pulse width (1000-2000us) to position (0-1023)
    // Speeduino typically outputs 1000-2000us PWM
    if (pwmPulseWidth >= 1000 && pwmPulseWidth <= 2000) {
      targetPosition = map(pwmPulseWidth, 1000, 2000, MIN_POSITION, MAX_POSITION);
      targetPosition = constrain(targetPosition, MIN_POSITION, MAX_POSITION);
    }
  }
}

// PID Controller
int calculatePID(int setpoint, int input) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds
  lastTime = now;
  
  // Calculate error
  float error = setpoint - input;
  
  // Apply deadband to prevent oscillation
  if (abs(error) <= DEADBAND) {
    error = 0;
  }
  
  // Proportional term
  float proportional = Kp * error;
  
  // Integral term (with windup protection)
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100); // Prevent integral windup
  float integralTerm = Ki * integral;
  
  // Derivative term
  float derivative = (error - previousError) / deltaTime;
  float derivativeTerm = Kd * derivative;
  
  // Calculate total output
  float output = proportional + integralTerm + derivativeTerm;
  
  // Constrain output to motor limits
  output = constrain(output, -255, 255);
  
  previousError = error;
  
  return (int)output;
}

// Drive motor with PWM and direction control
void driveMotor(int output) {
  if (output > 0) {
    // Move in positive direction
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    analogWrite(MOTOR_PWM_PIN, abs(output));
  } else if (output < 0) {
    // Move in negative direction
    digitalWrite(MOTOR_DIR_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, abs(output));
  } else {
    // Stop motor
    analogWrite(MOTOR_PWM_PIN, 0);
  }
}

// Print status information
void printStatus() {
  Serial.print("Target: ");
  Serial.print(targetPosition);
  Serial.print(" Current: ");
  Serial.print(currentPosition);
  Serial.print(" Error: ");
  Serial.print(targetPosition - currentPosition);
  Serial.print(" Motor: ");
  Serial.print(motorOutput);
  Serial.print(" PWM: ");
  Serial.print(pwmPulseWidth);
  Serial.print("us");
  if (!motorEnabled) Serial.print(" [DISABLED]");
  Serial.println();
}

// Handle serial commands for tuning and control
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("KP=")) {
      Kp = command.substring(3).toFloat();
      Serial.println("Kp set to " + String(Kp));
    }
    else if (command.startsWith("KI=")) {
      Ki = command.substring(3).toFloat();
      Serial.println("Ki set to " + String(Ki));
    }
    else if (command.startsWith("KD=")) {
      Kd = command.substring(3).toFloat();
      Serial.println("Kd set to " + String(Kd));
    }
    else if (command == "ENABLE") {
      motorEnabled = true;
      Serial.println("Motor enabled");
    }
    else if (command == "DISABLE") {
      motorEnabled = false;
      analogWrite(MOTOR_PWM_PIN, 0);
      Serial.println("Motor disabled");
    }
    else if (command == "STATUS") {
      Serial.println("=== IAC Controller Status ===");
      Serial.println("Target: " + String(targetPosition));
      Serial.println("Current: " + String(currentPosition));
      Serial.println("Motor Output: " + String(motorOutput));
      Serial.println("PWM Input: " + String(pwmPulseWidth) + "us");
      Serial.println("PID: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
      Serial.println("Motor: " + String(motorEnabled ? "ENABLED" : "DISABLED"));
    }
    else if (command == "HELP") {
      Serial.println("Available commands:");
      Serial.println("KP=value  - Set proportional gain");
      Serial.println("KI=value  - Set integral gain");
      Serial.println("KD=value  - Set derivative gain");
      Serial.println("ENABLE    - Enable motor control");
      Serial.println("DISABLE   - Disable motor control");
      Serial.println("STATUS    - Show detailed status");
      Serial.println("HELP      - Show this help");
    }
    else {
      Serial.println("Unknown command. Type HELP for available commands.");
    }
  }
}
