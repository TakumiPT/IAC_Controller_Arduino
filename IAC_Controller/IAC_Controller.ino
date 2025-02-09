#include <Arduino.h> // Include the Arduino library

// Define the pins used in the circuit
const int potPin = A0; // Pin where the potentiometer is connected
const int motorPin1 = 8; // Input pin 1 on the L293D motor driver
const int motorPin2 = 7; // Input pin 2 on the L293D motor driver
const int enablePin = 9; // Enable pin on the L293D motor driver

// Define constants
int targetPositionPercent = 100; // Target position in percentage (0-100)
const int tolerance = 15; // Tolerance value for the potentiometer reading
const int maxMotorSpeed = 255; // Maximum motor speed
const int maxPotentiometerPercent = 60; // Maximum potentiometer percentage the motor can move

// Function to map percentage to potentiometer value
int mapPercentToPotValue(int percent) {
  return map(percent, 0, 100, 0, 1023);
}

// Function to map target position percentage to potentiometer percentage
int mapTargetToPotPercent(int targetPercent) {
  return map(targetPercent, 0, 100, 0, maxPotentiometerPercent);
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(enablePin, LOW);
}

void setup() {
  // Set the pin modes
  pinMode(motorPin1, OUTPUT); // Set motor pin 1 as output
  pinMode(motorPin2, OUTPUT); // Set motor pin 2 as output
  pinMode(enablePin, OUTPUT); // Set enable pin as output

  Serial.begin(9600); // Initialize serial communication at 9600 bits per second
  Serial.println("Enter target position percentage (0-100):");

  // Initialize motor direction
  stopMotor();
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    int incomingValue = Serial.parseInt();
    // Check if the value is within the valid range
    if (incomingValue >= 0 && incomingValue <= 100) {
      targetPositionPercent = incomingValue;
      Serial.print("Target Position Percentage set to: ");
      Serial.println(targetPositionPercent);
    } else {
      Serial.println("Invalid value. Please enter a value between 0 and 100.");
    }
  }

  int potValue = analogRead(potPin); // Read the potentiometer value
  int targetPotPercent = mapTargetToPotPercent(targetPositionPercent); // Map target position to potentiometer percentage
  int targetPosition = mapPercentToPotValue(targetPotPercent); // Map target potentiometer percentage to potentiometer value
  int currentPercentage = map(potValue, 0, 1023, 0, 100); // Map potentiometer value to percentage

  Serial.print("Current Percentage: ");
  Serial.print(currentPercentage); // Print the current percentage to the serial monitor

  if (potValue < targetPosition - tolerance) { // If the current position is less than the target position
    digitalWrite(motorPin1, HIGH); // Set motor direction to forward
    digitalWrite(motorPin2, LOW);
    digitalWrite(enablePin, HIGH); // Enable motor
    Serial.print(" | Enable Pin: ON | Motor Direction: Forward");
  } else if (potValue > targetPosition + tolerance) { // If the current position is greater than the target position
    digitalWrite(motorPin1, LOW); // Set motor direction to backward
    digitalWrite(motorPin2, HIGH);
    digitalWrite(enablePin, HIGH); // Enable motor
    Serial.print(" | Enable Pin: ON | Motor Direction: Backward");
  } else { // If the current position is within the target range
    stopMotor(); // Stop the motor
    Serial.print(" | Enable Pin: OFF | Motor Direction: Stopped");
  }

  Serial.println(); // Move to the next line
  delay(100); // Add a small delay to stabilize the motor control
}
