#include <SPI.h>
#include <Arduino.h>
#include "TMC262.c"

// Define the SPI pins
#define TMC_MISO 1  // SDO -> MISO on ESP32
#define TMC_MOSI 2  // SDI -> MOSI on ESP32
#define TMC_SCK  3// SCK -> SCK on ESP32
#define TMC_CS   4   // CSN -> CS/SS on ESP32

#define STEP_PIN 5   // GPIO for STEP
#define DIR_PIN  6   // GPIO for DIR

const int stepCount = 50;  // Number of steps for a small move
const int stepDelay = 500; // Delay in microseconds (controls speed)

void setup() {
  // Initialize SPI with custom pins
  SPI.begin(TMC_SCK, TMC_MISO, TMC_MOSI, TMC_CS);
  
  // Configure Step/Dir pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Set default direction
  digitalWrite(DIR_PIN, HIGH); // Set direction to forward

  // Initialize the TMC262
  tmc262_initMotorDrivers();

  // Configure TMC262 settings (example: 16 microsteps)
  tmc262_setStepDirMStepRes(4); // 16 microsteps
  tmc262_setStepDirInterpolation(1); // Enable interpolation
}

void loop() {
  // Move the motor a small amount
  moveMotor(stepCount, stepDelay);

  // Wait a bit
  delay(1000);

  // Change direction and move back
  digitalWrite(DIR_PIN, LOW); // Reverse direction
  moveMotor(stepCount, stepDelay);

  // Wait again
  delay(1000);
}
void moveMotor(int steps, int delayMicroseconds) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH); // Step pulse HIGH
    delayMicroseconds(delayMicroseconds); // Short delay
    digitalWrite(STEP_PIN, LOW);  // Step pulse LOW
    delayMicroseconds(delayMicroseconds); // Short delay
  }
}