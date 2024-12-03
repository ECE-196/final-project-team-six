// Pin definitions for stepper motors
#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define STEP_PIN_2 4
#define DIR_PIN_2 5

// Constants for stepper motor control
#define STEPS_PER_REV 200  // Number of steps per revolution for the stepper motor
#define STEP_ANGLE 1.8     // Step angle in degrees

// Arm dimensions
#define L1 304.8           // Length of the first link
#define L2 304.8           // Length of the second link

// Target coordinates (initialize to home position)
double targetX = 200.0;
double targetY = 200.0;

void setup() {
  // Initialize motor pins as outputs
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Use W (up), S (down), A (left), D (right) to move the arm.");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read the command input

    // Adjust target coordinates based on command
    switch (command) {
      case 'W':  // Move up
        targetY += 10;
        break;
      case 'S':  // Move down
        targetY -= 10;
        break;
      case 'A':  // Move left
        targetX -= 10;
        break;
      case 'D':  // Move right
        targetX += 10;
        break;
      default:
        Serial.println("Invalid command. Use W, S, A, D.");
        return;
    }

    // Calculate new angles based on target coordinates
    double theta1, theta2;
    if (calculateAngles(targetX, targetY, &theta1, &theta2)) {
      // Convert angles from radians to degrees
      theta1 = theta1 * 180 / M_PI;
      theta2 = theta2 * 180 / M_PI;

      Serial.print("Moving to position: X = ");
      Serial.print(targetX);
      Serial.print(", Y = ");
      Serial.println(targetY);
      Serial.print("Target angles: Theta1 = ");
      Serial.print(theta1);
      Serial.print(", Theta2 = ");
      Serial.println(theta2);

      // Calculate the number of steps for each motor
      int steps1 = (int)(theta1 / STEP_ANGLE);
      int steps2 = (int)(theta2 / STEP_ANGLE);

      // Move stepper motors to the target position
      moveStepper(STEP_PIN_1, DIR_PIN_1, abs(steps1), steps1 > 0);
      moveStepper(STEP_PIN_2, DIR_PIN_2, abs(steps2), steps2 > 0);
    } else {
      Serial.println("Target position is unreachable.");
    }
  }
}

// Function to move a stepper motor
void moveStepper(int stepPin, int dirPin, int steps, int direction) {
  digitalWrite(dirPin, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);  // Adjust speed with this delay
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}

// Inverse kinematics calculation
bool calculateAngles(double x, double y, double* theta1, double* theta2) {
  double d = sqrt(x * x + y * y);
  if (d > (L1 + L2) || d < fabs(L1 - L2)) {
    return false;  // Target position is unreachable
  }

  double cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  *theta2 = atan2(sqrt(1 - cosTheta2 * cosTheta2), cosTheta2);
  double sinTheta2 = sin(*theta2);

  *theta1 = atan2(y, x) - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
  return true;  // Success
}
