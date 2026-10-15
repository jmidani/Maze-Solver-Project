#include <Arduino.h>

// Motor Driver Pins (L298N)
#define MOTOR_L_PWM 5
#define MOTOR_L_IN1 6
#define MOTOR_L_IN2 7

#define MOTOR_R_PWM 9
#define MOTOR_R_IN3 10
#define MOTOR_R_IN4 11

const uint8_t SENSOR_PINS[5] = {32, 24, 26, 28, 30};
#define STATUS_LED 13

// Sensor & Motor Settings
const bool SENSOR_TEST_MODE = false;
const bool LINE_IS_BLACK = true;

const bool INVERT_L = true;
const bool INVERT_R = false;

const float TRIM_LEFT = 1.00f;
const float TRIM_RIGHT = 1.00f;

// Speed Controls
const int SPEED_BASE = 75;
const int SPEED_MIN = 40;
const int SPEED_MAX = 110;
const int SPEED_TURN = 80;

// Tuning Parameters
const float KP_GAIN = 33.0f;
const float LEFT_BIAS = 1.25f;

// Timing Thresholds (ms)
const unsigned long TIMEOUT_TURN = 1200;
const unsigned long TIMEOUT_CLEAR = 700;
const unsigned long LOCKOUT_JUNCTION = 350;
const unsigned long CONFIRM_CLEAR = 100;
const unsigned long UTURN_MIN_TIME = 500;
const unsigned long UTURN_MAX_TIME = 2500;
const unsigned long PRINT_INTERVAL = 150;

// Global State Variables
bool sensorStates[5] = {false};
float previousError = 0.0f;
unsigned long lastPrintMs = 0;

bool isLockedJunction = false;
unsigned long lockStartTime = 0;
unsigned long clearStartTime = 0;
bool isClearTimerActive = false;

// ================= MOTOR CONTROL =================

void driveLeft(int speed) {
  speed = (int)(speed * TRIM_LEFT);
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (INVERT_L) speed = -speed;

  if (speed >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
  }
  analogWrite(MOTOR_L_PWM, abs(speed));
}

void driveRight(int speed) {
  speed = (int)(speed * TRIM_RIGHT);
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  if (INVERT_R) speed = -speed;

  if (speed >= 0) {
    digitalWrite(MOTOR_R_IN3, HIGH);
    digitalWrite(MOTOR_R_IN4, LOW);
  } else {
    digitalWrite(MOTOR_R_IN3, LOW);
    digitalWrite(MOTOR_R_IN4, HIGH);
  }
  analogWrite(MOTOR_R_PWM, abs(speed));
}

void setSpeeds(int leftSpd, int rightSpd) {
  driveLeft(leftSpd);
  driveRight(rightSpd);
}

void halt() {
  setSpeeds(0, 0);
}

// ================= SENSOR PROCESSING =================

void pollSensors() {
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(SENSOR_PINS[i]);
    sensorStates[i] = (LINE_IS_BLACK ? (raw == LOW) : (raw == HIGH));
  }
}

void displaySensors() {
  Serial.print("Line Status: [ ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorStates[i] ? "1 " : "0 ");
  }
  Serial.println("]");
}

float getLineOffset() {
  float weightedPos = 0.0f;
  int activeSensors = 0;
  const float positions[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

  for (int i = 0; i < 5; i++) {
    if (sensorStates[i]) {
      weightedPos += positions[i];
      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    return previousError;
  }

  return weightedPos / activeSensors;
}

// ================= STATE CHECKS =================

bool checkCrossroad() {
  return sensorStates[0] && sensorStates[4];
}

bool centerOnLine() {
  return sensorStates[2];
}

bool hasLeftBranch() {
  return sensorStates[0] || sensorStates[1];
}

bool hasRightBranch() {
  return sensorStates[3] || sensorStates[4];
}

bool allSensorsWhite() {
  for (int i = 0; i < 5; i++) {
    if (sensorStates[i]) return false;
  }
  return true;
}

bool allSensorsBlack() {
  for (int i = 0; i < 5; i++) {
    if (!sensorStates[i]) return false;
  }
  return true;
}

// Lockout tracking so crossroad isn't re-triggered
void processJunctionLock() {
  if (!isLockedJunction) return;

  if (millis() - lockStartTime < LOCKOUT_JUNCTION) return;

  if (!checkCrossroad()) {
    if (!isClearTimerActive) {
      clearStartTime = millis();
      isClearTimerActive = true;
    }

    if (millis() - clearStartTime >= CONFIRM_CLEAR) {
      isLockedJunction = false;
      isClearTimerActive = false;
      Serial.println("Crossroad cleared.");
    }
  } else {
    isClearTimerActive = false;
  }
}

// ================= TURNING LOGIC =================

bool spinLeftToLine() {
  unsigned long t0 = millis();

  // Rotate until middle sensor clears line
  while (true) {
    pollSensors();
    if (!centerOnLine()) break;
    setSpeeds(SPEED_TURN, -SPEED_TURN);

    if (millis() - t0 >= TIMEOUT_CLEAR) {
      halt();
      Serial.println("Left turn: center clear timeout.");
      return false;
    }
  }

  // Continue rotating until middle sensor re-acquires line
  t0 = millis();
  while (true) {
    pollSensors();
    if (centerOnLine()) break;
    setSpeeds(SPEED_TURN, -SPEED_TURN);

    if (millis() - t0 >= TIMEOUT_TURN) {
      halt();
      Serial.println("Left turn: re-acquire timeout.");
      return false;
    }
  }

  halt();
  delay(30);
  return true;
}

bool spinRightToLine() {
  unsigned long t0 = millis();

  while (true) {
    pollSensors();
    if (!centerOnLine()) break;
    setSpeeds(-SPEED_TURN, SPEED_TURN);

    if (millis() - t0 >= TIMEOUT_CLEAR) {
      halt();
      Serial.println("Right turn: center clear timeout.");
      return false;
    }
  }

  t0 = millis();
  while (true) {
    pollSensors();
    if (centerOnLine()) break;
    setSpeeds(-SPEED_TURN, SPEED_TURN);

    if (millis() - t0 >= TIMEOUT_TURN) {
      halt();
      Serial.println("Right turn: re-acquire timeout.");
      return false;
    }
  }

  halt();
  delay(30);
  return true;
}

bool executeUTurn() {
  Serial.println("Dead end: Executing U-turn.");
  unsigned long t0 = millis();

  while (millis() - t0 < UTURN_MIN_TIME) {
    setSpeeds(-SPEED_TURN, SPEED_TURN);
  }

  while (true) {
    pollSensors();
    if (centerOnLine()) break;
    setSpeeds(-SPEED_TURN, SPEED_TURN);

    if (millis() - t0 >= UTURN_MAX_TIME) {
      halt();
      Serial.println("U-turn timeout.");
      return false;
    }
  }

  halt();
  delay(30);
  return true;
}

void handleJunction() {
  Serial.println("Crossroad detected.");

  setSpeeds(SPEED_BASE, SPEED_BASE);
  delay(70);

  pollSensors();

  bool leftOpt = hasLeftBranch();
  bool rightOpt = hasRightBranch();
  bool straightOpt = centerOnLine();

  displaySensors();

  bool completed = false;

  // Decision Priority: Left -> Right -> Straight -> U-Turn
  if (leftOpt) {
    Serial.println("Turn: LEFT");
    completed = spinLeftToLine();
  } else if (rightOpt) {
    Serial.println("Turn: RIGHT");
    completed = spinRightToLine();
  } else if (straightOpt) {
    Serial.println("Turn: STRAIGHT");
    setSpeeds(SPEED_BASE, SPEED_BASE);
    delay(100);
    completed = true;
  } else {
    completed = executeUTurn();
  }

  if (completed) {
    isLockedJunction = true;
    lockStartTime = millis();
    isClearTimerActive = false;
    Serial.println("Intersection completed.");
  } else {
    halt();
    Serial.println("Navigation failed.");
  }
}

void recoverLine() {
  Serial.println("Line lost: Recovering...");
  unsigned long t0 = millis();

  int lSpeed = (previousError < 0) ? SPEED_TURN : -SPEED_TURN;
  int rSpeed = -lSpeed;

  while (true) {
    pollSensors();
    if (!allSensorsWhite()) break;

    setSpeeds(lSpeed, rSpeed);

    if (millis() - t0 >= TIMEOUT_TURN) {
      halt();
      Serial.println("Search timeout.");
      return;
    }

    if (millis() - lastPrintMs >= PRINT_INTERVAL) {
      lastPrintMs = millis();
      Serial.println("Searching...");
    }
  }

  halt();
  delay(30);
  Serial.println("Line re-acquired.");
}

// ================= SETUP & MAIN =================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);

  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT);
  pinMode(MOTOR_R_IN4, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  halt();

  // LED Blink Startup Sequence
  for (int k = 0; k < 6; k++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(250);
    digitalWrite(STATUS_LED, LOW);
    delay(250);
  }

  Serial.println("Robot Ready.");
}

void loop() {
  pollSensors();

  digitalWrite(STATUS_LED, !allSensorsWhite());

  if (millis() - lastPrintMs >= PRINT_INTERVAL) {
    lastPrintMs = millis();
    displaySensors();
  }

  if (SENSOR_TEST_MODE) {
    halt();
    return;
  }

  processJunctionLock();

  if (allSensorsWhite()) {
    recoverLine();
    return;
  }

  if (checkCrossroad() && !isLockedJunction) {
    handleJunction();
    return;
  }

  if (allSensorsBlack()) {
    setSpeeds(SPEED_BASE, SPEED_BASE);
    return;
  }

  // P-Controller calculation
  float currentErr = getLineOffset();
  previousError = currentErr;

  float scaledErr = currentErr;
  if (currentErr < 0) {
    scaledErr *= LEFT_BIAS;
  }

  int steeringCorrection = (int)(KP_GAIN * scaledErr);

  int lDrive = constrain(SPEED_BASE - steeringCorrection, SPEED_MIN, SPEED_MAX);
  int rDrive = constrain(SPEED_BASE + steeringCorrection, SPEED_MIN, SPEED_MAX);

  setSpeeds(lDrive, rDrive);

  delay(2);
}