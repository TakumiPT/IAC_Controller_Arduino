#include <Arduino.h>

// Variables to store the duration of the high and low states of the PWM signal
unsigned long highDuration;
unsigned long lowDuration;

// Variable to store the calculated duty cycle as a percentage
float dutyCycle;

// Define the pin connected to the open-drain output
const int pwmPin = 2;

void setup() {
  // Configure the pwmPin as an input with an internal pull-up resistor
  pinMode(pwmPin, INPUT_PULLUP);
  
  // Initialize serial communication at a baud rate of 9600
  Serial.begin(9600);
}

void loop() {
  // Measure the duration of the high state of the PWM signal
  highDuration = pulseIn(pwmPin, HIGH);
  
  // Measure the duration of the low state of the PWM signal
  lowDuration = pulseIn(pwmPin, LOW);
  
  // Calculate the total period of the PWM signal
  unsigned long period = highDuration + lowDuration;
  
  // Calculate the duty cycle as a percentage
  if (period > 0) {
    dutyCycle = (highDuration / (float)period) * 100.0;
  } else {
    dutyCycle = 0;
  }

  // Print the high duration, low duration, and duty cycle to the serial monitor
  Serial.print("High Duration: ");
  Serial.print(highDuration);
  Serial.print(" Low Duration: ");
  Serial.print(lowDuration);
  Serial.print(" Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.println("%");
  
  // Wait for 1 second before repeating the loop
  delay(1000);
}