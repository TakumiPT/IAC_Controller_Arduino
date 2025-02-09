#include <Arduino.h>

const int pwmPin = 2; // Define the pin connected to the PWM output
const int analogPin = A0; // Define the analog pin to read the filtered PWM signal
const int numReadings = 10; // Number of readings to average

int readings[numReadings]; // Array to store the readings
int readIndex = 0; // Index of the current reading
int total = 0; // Sum of the readings
int average = 0; // Average of the readings

void setup() {
  pinMode(pwmPin, INPUT_PULLUP); // Configure the pwmPin as an input with an internal pull-up resistor
  Serial.begin(9600); // Initialize serial communication at a baud rate of 9600

  // Initialize the readings array to 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  // Subtract the last reading from the total
  total = total - readings[readIndex];

  // Read the current value from the analog pin
  readings[readIndex] = analogRead(analogPin);

  // Add the current reading to the total
  total = total + readings[readIndex];

  // Advance to the next position in the array
  readIndex = readIndex + 1;

  // If we're at the end of the array, wrap around to the beginning
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // Calculate the average of the readings
  average = total / numReadings;

  // Convert the average to a voltage (assuming a 5V reference)
  float voltage = average * (5.0 / 1023.0);

  // Print the average and voltage to the serial monitor
  Serial.print("Average: ");
  Serial.print(average);
  Serial.print(" Voltage: ");
  Serial.println(voltage);

  delay(100); // Wait for 100 milliseconds before repeating the loop
}