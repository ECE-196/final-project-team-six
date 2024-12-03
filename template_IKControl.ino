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

void setup() {
  // Initialize motor pins as outputs
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Enter target X and Y coordinates:");
}

void loop() {
  if (Serial.available() > 0) {
    // Read input coordinates
    String input = Serial.readStringUntil('\n');
    double x, y;
    sscanf(input.c_str(), "%lf %lf", &x, &y);

    // Move the robotic arm to the specified position
    moveToPosition(x, y);
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

void moveToPosition(double x, double y) {
  double theta1, theta2;

  if (!calculateAngles(x, y, &theta1, &theta2)) {
    Serial.println("Position is unreachable.");
    return;
  }

  // Convert angles from radians to degrees
  theta1 = theta1 * 180 / M_PI;
  theta2 = theta2 * 180 / M_PI;

  Serial.print("Target angles: Theta1 = ");
  Serial.print(theta1);
  Serial.print(", Theta2 = ");
  Serial.println(theta2);

  // Calculate the number of steps required for each motor
  int steps1 = (int)(theta1 / STEP_ANGLE);
  int steps2 = (int)(theta2 / STEP_ANGLE);

  // Move stepper motors to the target positions
  moveStepper(STEP_PIN_1, DIR_PIN_1, abs(steps1), steps1 > 0);
  moveStepper(STEP_PIN_2, DIR_PIN_2, abs(steps2), steps2 > 0);
}
