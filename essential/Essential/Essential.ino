/*
 * Essential: the right wheel's neutral pulse changed from 1488 to 1495 because it crept forward at 1488.
 * Part 1 blue route: handle one of four start orientations once, follow the corridor, then stop at a front wall.
 * Layout: 1. Global parameters and variables (original values). 2. Original controller (condensed and annotated).
 */
#include <Servo.h>  // Servo control library

// ======================= 1. Global parameters and variables =======================
// ---------- Pins: left and right motors and left, front and right IR emitters, receivers and LEDs ----------
const int PORT_DRIVE_LEFT = 13;
const int PORT_DRIVE_RIGHT = 12;
const int PORT_EMITTER_LEFT = 10,  PORT_RECEIVER_LEFT = 11, PORT_INDICATOR_LEFT = A2;
const int PORT_EMITTER_FRONT = 6,  PORT_RECEIVER_FRONT = 7, PORT_INDICATOR_FRONT = A1;
const int PORT_EMITTER_RIGHT = 2,  PORT_RECEIVER_RIGHT = 3, PORT_INDICATOR_RIGHT = A0;

// ---------- Motor pulses in microseconds: the right wheel is reversed, so forward pulses are below neutral ----------
const int PULSE_LEFT_STOP = 1494;
const int PULSE_RIGHT_STOP = 1495;    // Remeasured on Oct 29 (previously 1488)
const int PULSE_LEFT_FORWARD = 1544;    // Left wheel at low forward speed (neutral + 50)
const int PULSE_RIGHT_FORWARD = 1445;   // Right wheel at low forward speed (neutral - 50, recalculated from 1438)

// ---------- Parameters to update after hardware tests ----------
// The target distance is for reference. The sensor returns a score from 0 to 6 (6 = no detection, lower = closer), rather than a distance in centimetres.
const float SIDE_CORRECTION_CM = 5.0;
const float WALL_DISTANCE_CM = 8.0;

// Keep the original score thresholds for now. The proposed 5 cm and 8 cm cutoffs have not been verified.
const int SIDE_CLOSE_SCORE_MAX = 1;     // Side score <= 1 means the robot is too close to that wall
const int WALL_SCORE_MAX = 5;           // Front score <= 5 means a wall is ahead

// Initial fixed speed adjustment for steering, to be tuned.
const int CORRECTION_BOOST_US = 30;

// Initial 180-degree single-wheel pivot estimate: 180 * 2 * 820 / 90. Calibrate the left and right durations separately.
const unsigned long LEFT_WHEEL_180_MS = 3280;
const unsigned long RIGHT_WHEEL_180_MS = 3280;

// ---------- Motor objects, sensor readings and runtime state ----------
Servo motorLeft;
Servo motorRight;

int scoreLeft = 0;
int scoreFront = 0;
int scoreRight = 0;

// Pulse widths for corridor driving. Gradually return to their baseline after a steering correction.
int leftPulse = PULSE_LEFT_FORWARD;
int rightPulse = PULSE_RIGHT_FORWARD;
bool startupComplete = false;           // Whether one of the four starting cases has already been handled

// ======================= 2. Original controller (condensed) =======================
void setup() {
  // Set up all three IR groups: emitters as outputs, receivers as inputs, and LEDs as outputs initially off.
  pinMode(PORT_EMITTER_LEFT, OUTPUT);  pinMode(PORT_RECEIVER_LEFT, INPUT);  pinMode(PORT_INDICATOR_LEFT, OUTPUT);
  pinMode(PORT_EMITTER_FRONT, OUTPUT); pinMode(PORT_RECEIVER_FRONT, INPUT); pinMode(PORT_INDICATOR_FRONT, OUTPUT);
  pinMode(PORT_EMITTER_RIGHT, OUTPUT); pinMode(PORT_RECEIVER_RIGHT, INPUT); pinMode(PORT_INDICATOR_RIGHT, OUTPUT);
  digitalWrite(PORT_INDICATOR_LEFT, LOW);
  digitalWrite(PORT_INDICATOR_FRONT, LOW);
  digitalWrite(PORT_INDICATOR_RIGHT, LOW);

  // Attach the servos and send neutral pulses first.
  motorLeft.attach(PORT_DRIVE_LEFT);
  motorLeft.writeMicroseconds(PULSE_LEFT_STOP);
  motorRight.attach(PORT_DRIVE_RIGHT);
  motorRight.writeMicroseconds(PULSE_RIGHT_STOP);

  Serial.begin(9600);
  Serial.println(F("Part1_Simple_v3"));   // Print the version over serial to verify which sketch was uploaded.
}

void loop() {
  // Read the sensors first. Left and right are relative to the robot.
  readSensors();
  bool frontWall = scoreFront <= WALL_SCORE_MAX;
  bool leftTooClose = scoreLeft <= SIDE_CLOSE_SCORE_MAX;
  bool rightTooClose = scoreRight <= SIDE_CLOSE_SCORE_MAX;

  // Distinguish four starting cases once: clear ahead means facing forward, and a detected front wall means facing back.
  if (!startupComplete) {
    if (!frontWall && (leftTooClose || rightTooClose)) {
      followCorridor();              // Cases 1 and 2: facing forward near a side wall, speed up the wheel closest to it.
    } else if (frontWall && leftTooClose) {
      turnAroundFromLeftWall();      // Case 3: facing back near the left wall, drive the left wheel and keep the right still to turn right.
    } else if (frontWall && rightTooClose) {
      turnAroundFromRightWall();     // Case 4: facing back near the right wall, drive the right wheel and keep the left still to turn left.
    }
    startupComplete = true;
    return;  // On the next iteration, read the sensors from the new position before checking for the finish.
  }

  // After handling the start, stop at a front wall and end the run. Reset or power-cycle to start again.
  if (frontWall) {
    stopRobot();
    while (true) {}
  }

  // If the path ahead is clear, continue forward with side-wall corrections as needed.
  followCorridor();
}

// Scan six frequencies from 38 to 43 kHz and return a score from 0 to 6.
// At each frequency, 0 means a reflection was detected and 1 means none was detected. A lower total indicates a closer wall.
int readIRScore(int emitter, int receiver) {
  int score = 0;
  for (long frequency = 38000; frequency <= 43000; frequency += 1000) {
    tone(emitter, frequency);
    delay(1);
    score += digitalRead(receiver);
    noTone(emitter);
    delay(1);
  }
  return score;
}

void readSensors() {
  scoreLeft = readIRScore(PORT_EMITTER_LEFT, PORT_RECEIVER_LEFT);
  scoreFront = readIRScore(PORT_EMITTER_FRONT, PORT_RECEIVER_FRONT);
  scoreRight = readIRScore(PORT_EMITTER_RIGHT, PORT_RECEIVER_RIGHT);
}

void stopRobot() {
  motorLeft.writeMicroseconds(PULSE_LEFT_STOP);
  motorRight.writeMicroseconds(PULSE_RIGHT_STOP);
}

// For a single-wheel U-turn, hold one wheel still, drive the other forward for ms milliseconds, then stop.
void pivot(int leftUs, int rightUs, unsigned long ms) {
  motorLeft.writeMicroseconds(leftUs);
  motorRight.writeMicroseconds(rightUs);
  delay(ms);
  stopRobot();
}

// Near the left wall, drive the left wheel and pivot right around the stationary right wheel.
void turnAroundFromLeftWall() { pivot(PULSE_LEFT_FORWARD, PULSE_RIGHT_STOP, LEFT_WHEEL_180_MS); }

// Near the right wall, drive the right wheel and pivot left around the stationary left wheel.
void turnAroundFromRightWall() { pivot(PULSE_LEFT_STOP, PULSE_RIGHT_FORWARD, RIGHT_WHEEL_180_MS); }

// During corridor driving, speed up the wheel nearest the wall. Once clear, gradually return to the baseline pulses.
void followCorridor() {
  if (scoreLeft <= SIDE_CLOSE_SCORE_MAX) {
    // Near the left wall, increase the left pulse only to steer right.
    leftPulse = PULSE_LEFT_FORWARD + CORRECTION_BOOST_US;
    rightPulse = PULSE_RIGHT_FORWARD;
  } else if (scoreRight <= SIDE_CLOSE_SCORE_MAX) {
    // Near the right wall, decrease the right pulse only to steer left.
    leftPulse = PULSE_LEFT_FORWARD;
    rightPulse = PULSE_RIGHT_FORWARD - CORRECTION_BOOST_US;
  } else {
    // Move each pulse 1 microsecond toward its forward baseline per iteration. This alone does not establish parallel alignment.
    if (leftPulse > PULSE_LEFT_FORWARD) leftPulse--;
    if (rightPulse < PULSE_RIGHT_FORWARD) rightPulse++;
  }
  motorLeft.writeMicroseconds(leftPulse);
  motorRight.writeMicroseconds(rightPulse);
}
