/*
 * Extension Part 1: condensed version of the controller with a global route flag that passed a robot test.
 * The right wheel's neutral pulse changed from 1488 to 1495 and the other right-wheel pulses follow that baseline. Thresholds, steering, timing and action order were kept.
 * At startup, detect and correct cases 5-10, then navigate in this priority order: right, straight, left.
 * At the first three-wall dead end, set the flag to 1 and turn 180 degrees. Stop at the second. Read L/M/R scores at 9600 baud.
 */
#include "Servo.h"

// ======================= Global parameters =======================
const int PORT_DRIVE_LEFT = 13, PORT_DRIVE_RIGHT = 12;                                // Servo pins
const int PORT_EMITTER_LEFT = 10, PORT_RECEIVER_LEFT = 11, PORT_INDICATOR_LEFT = A2;  // Left IR emitter, receiver and LED
const int PORT_EMITTER_FRONT = 6, PORT_RECEIVER_FRONT = 7, PORT_INDICATOR_FRONT = A1; // Front IR
const int PORT_EMITTER_RIGHT = 2, PORT_RECEIVER_RIGHT = 3, PORT_INDICATOR_RIGHT = A0; // Right IR
// Servo pulses in microseconds: the right wheel is reversed, so forward pulses are below neutral.
const int PULSE_NEUTRAL = 1494, PULSE_RIGHT_NEUTRAL = 1495;  // Left and right neutral pulses (right remeasured Oct 29, previously 1488)
const int PULSE_LEFT_CRUISE = PULSE_NEUTRAL + 175, PULSE_RIGHT_CRUISE = PULSE_RIGHT_NEUTRAL - 175;   // Cruise
const int PULSE_LEFT_CREEP = PULSE_NEUTRAL + 50, PULSE_RIGHT_CREEP = PULSE_RIGHT_NEUTRAL - 50;       // Slow forward motion
const int PULSE_LEFT_REVERSE = PULSE_NEUTRAL - 50, PULSE_RIGHT_REVERSE = PULSE_RIGHT_NEUTRAL + 50;   // Reverse pulse for spinning in place
const int PULSE_LEFT_BACKWARD = PULSE_NEUTRAL - 45, PULSE_RIGHT_BACKWARD = PULSE_RIGHT_NEUTRAL + 60; // Straight reverse (increase the right pulse if the rear drifts right)
// Maze navigation. IR scores range from 0 to 6: 6 means no wall, lower means closer, and about 3 is centred.
const int LIMIT_PATH_CLEAR = 5, LIMIT_RIGHT_CLEAR = 6, TARGET_SIDE_SCORE = 3;  // Front or left > 5 and right >= 6 indicate clear paths. The target score is for single-wall following.
const int WAIT_RIGHT_APPROACH_MS = 820, WAIT_LEFT_APPROACH_MS = 250, DURATION_EXIT_CRUISE_MS = 500;  // Forward travel before turns and after a right turn
const int DURATION_MAZE_LEFT_90_MS = 660, DURATION_MAZE_RIGHT_90_MS = 1000, DURATION_MAZE_TURN_AROUND_MS = 2000;  // Decrease if the robot turns too far.
// Cases 5 and 6: the near side scores <= 1 and the far side at least 3 higher. Cases 7 and 8 allow <= 2 because angled readings appear farther.
const int LIMIT_SIDE_NEAR = 1, LIMIT_SIDE_NEAR_ANGLED = 2, LIMIT_SIDE_DIFF = 3;
const unsigned long DISPLAY_MS = 5000;  // Show the LEDs and stay still for at least 5 seconds.
const float RECENTRE_OUT_DEG_SCENARIO5 = 30.0, RECENTRE_BACK_DEG_SCENARIO5 = 5.0;   // Single-wheel pivot angles for moving out and back into alignment in case 5
const float RECENTRE_OUT_DEG_SCENARIO6 = 15.0, RECENTRE_BACK_DEG_SCENARIO6 = 50.0;  // Case 6
const float RECENTRE_REVERSE_CM = 8.0, RECENTRE_MAX_CM = 15.0, SENSOR_AHEAD_CM = 5.0;  // Reverse distance, diagonal travel limit, and sensor offset ahead of the axle
const int DURATION_RIGHT_QUARTER_MS = 820, DURATION_LEFT_QUARTER_MS = 820;  // Single-wheel pivot conversion (90 degrees = 2 x 820 ms)
const float MS_PER_CM_CREEP = 100.0;          // Time to creep forward 1 cm
const bool CONTINUE_AFTER_START_FIX = true;   // true continues navigation after correction. false stops for inspection.
const bool DIAGNOSTIC_LEDS = true; const unsigned long DIAGNOSTIC_MS = 3000;  // Startup diagnostics: each side LED indicates a score <= 2, and the middle LED indicates a front wall.
// Cases 7 and 8: at about 30 degrees to a wall, scan 35-41 kHz in 500 Hz steps for 20 passes (260 readings). At least 5 hits indicate a wall.
const int FRONT_EDGE_ROUNDS = 20, LIMIT_FRONT_EDGE_HITS = 5; const unsigned long FLASH_MS = 1000;  // Flash 1 second on, 1 second off.
const float ALIGN_REVERSE_CM_SCENARIO7 = 6.0, ALIGN_REVERSE_CM_SCENARIO8 = 5.0;  // Reverse away from the wall first.
const float ALIGN_DEG_SCENARIO7 = 45.0, ALIGN_DEG_SCENARIO8 = 16.0, ALIGN_FORWARD_CM = 4.0;  // Pivot on one wheel to align, then move forward.
// Cases 9 and 10: at a front corner, the fine front score (0-13) is <= 2. Reverse diagonally on one wheel for 600 ms, then spin 150 degrees.
const int LIMIT_FRONT_CORNER_FINE = 2; const unsigned long CORNER_DIAGONAL_BACK_MS = 600; const float CORNER_SPIN_DEG = 150.0;

// ======================= Global variables =======================
Servo driveUnitLeft, driveUnitRight;
int scoreSideLeft = 0, scoreAhead = 0, scoreSideRight = 0;  // Current left, front and right scores
int flag = 0;                                               // 0 = outbound. 1 = return after turning at the first dead end.
bool startChecked = false, frontWallStart = false;          // Whether the starting case was checked and whether a front wall was detected at startup (used in cases 7 and 8)

// ======================= Main controller =======================
void setup() {
  pinMode(PORT_EMITTER_LEFT, OUTPUT);  pinMode(PORT_RECEIVER_LEFT, INPUT);  pinMode(PORT_INDICATOR_LEFT, OUTPUT);
  pinMode(PORT_EMITTER_FRONT, OUTPUT); pinMode(PORT_RECEIVER_FRONT, INPUT); pinMode(PORT_INDICATOR_FRONT, OUTPUT);
  pinMode(PORT_EMITTER_RIGHT, OUTPUT); pinMode(PORT_RECEIVER_RIGHT, INPUT); pinMode(PORT_INDICATOR_RIGHT, OUTPUT);
  showLeds(LOW, LOW, LOW); stopRobot();  // Leave the servos detached until the first move so the wheels stay still.
  Serial.begin(9600); Serial.println(F("Part1_GlobalFlag_v3")); delay(2000);
}
void loop() {
  readAllSensors();
  Serial.print(F("L=")); Serial.print(scoreSideLeft); Serial.print(F(" M=")); Serial.print(scoreAhead); Serial.print(F(" R=")); Serial.println(scoreSideRight);
  if (!startChecked) {  // Check cases 5-10 only at startup. Evaluate each independently so the higher number wins if multiple cases match.
    startChecked = true;
    frontWallStart = frontSeesWall();
    showStartDiagnostics();
    int s = 0;
    if (isCloseToWall(scoreSideLeft, scoreSideRight)) s = 5;                           // Close to the left wall
    if (isCloseToWall(scoreSideRight, scoreSideLeft)) s = 6;                           // Close to the right wall
    if (scoreSideLeft <= LIMIT_SIDE_NEAR_ANGLED && frontWallStart) s = 7;              // Angled toward the left wall
    if (scoreSideRight <= LIMIT_SIDE_NEAR_ANGLED && frontWallStart) s = 8;             // Angled toward the right wall
    if (scoreSideLeft <= LIMIT_SIDE_NEAR && canGoRight() && frontVeryClose()) s = 9;   // Stuck in the front-left corner
    if (scoreSideRight <= LIMIT_SIDE_NEAR && canGoLeft() && frontVeryClose()) s = 10;  // Stuck in the front-right corner
    Serial.print(F("Start scenario = ")); Serial.println(s);
    switch (s) {
      case 5: showLeds(HIGH, LOW, HIGH); holdStill(DISPLAY_MS); recentre(1); finishStartFix(false); return;      // Left and right LEDs. Shift right.
      case 6: showLeds(HIGH, HIGH, LOW); holdStill(DISPLAY_MS); recentre(-1); finishStartFix(false); return;     // Left and middle LEDs. Shift left.
      case 7: showLeds(HIGH, HIGH, HIGH); holdStill(DISPLAY_MS); backTurnForward(1); finishStartFix(true); return;  // All LEDs on. Turn right until parallel.
      case 8: flashLedsStill(LOW, LOW, HIGH); backTurnForward(-1); finishStartFix(true); return;                 // Flash the right LED. Turn left until parallel.
      case 9:   // Flash the middle LED. Reverse the left wheel to swing the rear right, then turn 150 degrees clockwise.
        flashLedsStill(LOW, HIGH, LOW); move(PULSE_LEFT_REVERSE, PULSE_RIGHT_NEUTRAL, CORNER_DIAGONAL_BACK_MS);
        spinRight((unsigned long)(CORNER_SPIN_DEG * DURATION_MAZE_RIGHT_90_MS / 90.0)); finishStartFix(false); return;
      case 10:  // Flash the middle and right LEDs. Reverse the right wheel to swing the rear left, then turn 150 degrees anticlockwise.
        flashLedsStill(LOW, HIGH, HIGH); move(PULSE_NEUTRAL, PULSE_RIGHT_REVERSE, CORNER_DIAGONAL_BACK_MS);
        spinLeft((unsigned long)(CORNER_SPIN_DEG * DURATION_MAZE_LEFT_90_MS / 90.0)); finishStartFix(false); return;
    }  // s = 0 means no startup case matched. Continue to maze navigation.
  }
  // LED codes for navigation: straight = right, right turn = middle, left turn = middle and right, dead end = left.
  if (canGoLeft() && canGoForward() && canGoRight()) { showLeds(LOW, LOW, LOW); haltRobot(); }  // Case 0: all three directions are open. Stop for the rest of the run.
  else if (canGoRight()) {   // Case 2: move to the junction centre, turn right 90 degrees, then move out.
    showLeds(LOW, HIGH, LOW);
    cruiseFor(WAIT_RIGHT_APPROACH_MS); stopRobot(); delay(100);
    spinRight(DURATION_MAZE_RIGHT_90_MS); cruiseFor(DURATION_EXIT_CRUISE_MS);
  }
  else if (canGoForward()) { showLeds(LOW, LOW, HIGH); driveForward(); wallControl(); }  // Case 1: drive straight with steering corrections.
  else if (canGoLeft()) {    // Case 3: advance slightly, turn left 90 degrees, then advance again.
    showLeds(LOW, HIGH, HIGH);
    driveForward(); delay(WAIT_LEFT_APPROACH_MS); stopRobot(); delay(100);
    spinLeft(DURATION_MAZE_LEFT_90_MS); driveForward(); delay(200);
  }
  else {                     // Case 4: walls on three sides. On the first encounter, set the flag and turn around. On the return encounter, stop.
    showLeds(HIGH, LOW, LOW);
    if (flag == 0) { flag = 1; spinRight(DURATION_MAZE_TURN_AROUND_MS); } else haltRobot();
  }
}

// ======================= IR sensors =======================
// Take one reading at a carrier frequency. Return 0 for a reflection and 1 for no reflection.
int irDetect(int emitter, int receiver, long hz) { tone(emitter, hz); delay(1); int s = digitalRead(receiver); noTone(emitter); delay(1); return s; }
// Standard reading: six frequencies from 38 to 43 kHz, giving a score from 0 to 6.
int irDistance(int emitter, int receiver) { int missed = 0; for (long hz = 38000; hz <= 43000; hz += 1000) missed += irDetect(emitter, receiver, hz); return missed; }
void readAllSensors() {
  scoreSideLeft = irDistance(PORT_EMITTER_LEFT, PORT_RECEIVER_LEFT); scoreAhead = irDistance(PORT_EMITTER_FRONT, PORT_RECEIVER_FRONT);
  scoreSideRight = irDistance(PORT_EMITTER_RIGHT, PORT_RECEIVER_RIGHT);
}
bool canGoRight() { return LIMIT_RIGHT_CLEAR <= scoreSideRight; }  bool canGoForward() { return LIMIT_PATH_CLEAR < scoreAhead; }
bool canGoLeft() { return LIMIT_PATH_CLEAR < scoreSideLeft; }

// ======================= Startup case detection =======================
// Cases 5 and 6: one side is close, the other is clearly farther away, and the path ahead is open.
bool isCloseToWall(int nearScore, int farScore) { return nearScore <= LIMIT_SIDE_NEAR && farScore - nearScore >= LIMIT_SIDE_DIFF && canGoForward(); }
// Cases 9 and 10: the front wall is very close. Average five sweeps over 13 frequencies from 38 to 50 kHz for a score from 0 to 13.
bool frontVeryClose() {
  int total = 0;
  for (int i = 0; i < 5; i++) for (long hz = 38000; hz <= 50000; hz += 1000) total += irDetect(PORT_EMITTER_FRONT, PORT_RECEIVER_FRONT, hz);
  Serial.print(F("Front fine = ")); Serial.println((total + 2) / 5);
  return (total + 2) / 5 <= LIMIT_FRONT_CORNER_FINE;
}
// Cases 7 and 8: the standard front reading detects a wall, or repeated readings at the edge of range reach LIMIT_FRONT_EDGE_HITS.
bool frontSeesWall() {
  if (!canGoForward()) return true;
  int hits = 0;
  for (int round = 0; round < FRONT_EDGE_ROUNDS; round++)
    for (long hz = 35000; hz <= 41000; hz += 500) if (irDetect(PORT_EMITTER_FRONT, PORT_RECEIVER_FRONT, hz) == 0) hits++;
  Serial.print(F("Front edge hits = ")); Serial.println(hits);
  return hits >= LIMIT_FRONT_EDGE_HITS;
}
// Use the three LEDs at startup to check cases 7 and 8 without a serial connection.
void showStartDiagnostics() {
  bool leftNear = scoreSideLeft <= LIMIT_SIDE_NEAR_ANGLED, rightNear = scoreSideRight <= LIMIT_SIDE_NEAR_ANGLED; if (!DIAGNOSTIC_LEDS) return;
  showLeds(leftNear, frontWallStart, rightNear); delay(DIAGNOSTIC_MS); showLeds(LOW, LOW, LOW); delay(500);
}

// ======================= Startup manoeuvres =======================
// Cases 5 and 6: direction = 1 shifts right from the left wall, and -1 shifts left.
void recentre(int direction) {
  if (direction > 0) arcRight(RECENTRE_OUT_DEG_SCENARIO5); else arcLeft(RECENTRE_OUT_DEG_SCENARIO6);  // 1. Pivot out on one wheel.
  drive(PULSE_LEFT_CREEP, PULSE_RIGHT_CREEP);  // 2. Creep diagonally until the near-side score reaches the far-side score and the sensors are centred.
  unsigned long startTimestamp = millis(); float travelled = 0; bool centred = false;
  while (travelled < RECENTRE_MAX_CM) {
    readAllSensors();
    travelled = (millis() - startTimestamp) / MS_PER_CM_CREEP;
    if ((direction > 0 ? scoreSideLeft : scoreSideRight) >= (direction > 0 ? scoreSideRight : scoreSideLeft)) { centred = true; break; }
  }
  if (centred) delay((unsigned long)(SENSOR_AHEAD_CM * MS_PER_CM_CREEP));  // 3. Move a little farther so the robot's centre also reaches the corridor centre.
  stopRobot(); delay(100);
  if (direction > 0) arcLeft(RECENTRE_BACK_DEG_SCENARIO5); else arcRight(RECENTRE_BACK_DEG_SCENARIO6);  // 4. Pivot back until parallel.
  move(PULSE_LEFT_BACKWARD, PULSE_RIGHT_BACKWARD, (unsigned long)(RECENTRE_REVERSE_CM * MS_PER_CM_CREEP));  // 5. Reverse a short distance.
}
// Cases 7 and 8: reverse away from the wall, pivot on one wheel to align, then drive forward. direction = 1 means facing the left wall, so turn right.
void backTurnForward(int direction) {
  float reverseCm = (direction > 0) ? ALIGN_REVERSE_CM_SCENARIO7 : ALIGN_REVERSE_CM_SCENARIO8;
  move(PULSE_LEFT_BACKWARD, PULSE_RIGHT_BACKWARD, (unsigned long)(reverseCm * MS_PER_CM_CREEP));
  if (direction > 0) arcRight(ALIGN_DEG_SCENARIO7); else arcLeft(ALIGN_DEG_SCENARIO8);
  move(PULSE_LEFT_CREEP, PULSE_RIGHT_CREEP, (unsigned long)(ALIGN_FORWARD_CM * MS_PER_CM_CREEP));
}
// After startup correction, continue navigation or stop. If recheck is true after cases 7 and 8, check again for an offset and recentre as in cases 5 and 6.
void finishStartFix(bool recheck) { if (!CONTINUE_AFTER_START_FIX) haltRobot(); if (recheck) startChecked = false; }

// ======================= Steering and motion =======================
// Cruise for ms milliseconds while reading the sensors and correcting the heading.
void cruiseFor(unsigned long ms) { unsigned long t0 = millis(); while (millis() - t0 < ms) { readAllSensors(); driveForward(); wallControl(); } }
// Corridor steering (gain 30): use the side-score difference with two walls, or TARGET_SIDE_SCORE with one. Positive adj steers right and negative adj steers left.
void wallControl() {
  const float GAIN_SIDE_CONTROL = 30.0;
  int adj = 0;
  if (scoreSideLeft <= LIMIT_PATH_CLEAR && scoreSideRight <= LIMIT_PATH_CLEAR) adj = GAIN_SIDE_CONTROL * (scoreSideRight - scoreSideLeft);
  else if (scoreSideLeft < 4 && scoreSideRight > LIMIT_PATH_CLEAR) { adj = GAIN_SIDE_CONTROL * (TARGET_SIDE_SCORE - scoreSideLeft); adj = adj * 0.7; }  // Left wall only: use 70 percent of the correction, based on a hardware test.
  else if (scoreSideRight < 4 && scoreSideLeft > LIMIT_PATH_CLEAR) adj = -(int)(GAIN_SIDE_CONTROL * (TARGET_SIDE_SCORE - scoreSideRight));
  if (adj != 0) { driveUnitLeft.writeMicroseconds(PULSE_LEFT_CRUISE + adj); driveUnitRight.writeMicroseconds(PULSE_RIGHT_CRUISE + adj); }  // 0 means keep the cruise pulses.
}
void showLeds(int l, int m, int r) { digitalWrite(PORT_INDICATOR_LEFT, l); digitalWrite(PORT_INDICATOR_FRONT, m); digitalWrite(PORT_INDICATOR_RIGHT, r); }
void releaseServos() { stopRobot(); driveUnitLeft.detach(); driveUnitRight.detach(); }  // Detach the servo signals to prevent wheel creep.
void holdStill(unsigned long ms) { releaseServos(); delay(ms); }
void haltRobot() { releaseServos(); for (;;) delay(1000); }  // Stop until reset.
// Hold still for DISPLAY_MS and flash the selected LEDs together, 1 second on and 1 second off.
void flashLedsStill(int l, int m, int r) {
  showLeds(LOW, LOW, LOW); releaseServos();
  for (unsigned long t = 0; t < DISPLAY_MS; t += 2 * FLASH_MS) { showLeds(l, m, r); delay(FLASH_MS); showLeds(LOW, LOW, LOW); delay(FLASH_MS); }
}
// Servos start detached and are detached while stationary. Attach before writing movement pulses.
void drive(int leftUs, int rightUs) {
  if (!driveUnitLeft.attached()) driveUnitLeft.attach(PORT_DRIVE_LEFT);
  if (!driveUnitRight.attached()) driveUnitRight.attach(PORT_DRIVE_RIGHT);
  driveUnitLeft.writeMicroseconds(leftUs); driveUnitRight.writeMicroseconds(rightUs);
}
// Drive with the given pulses for ms milliseconds, then stop for 0.1 seconds. This is used for turns and reversing.
void move(int leftUs, int rightUs, unsigned long ms) { drive(leftUs, rightUs); delay(ms); stopRobot(); delay(100); }
void stopRobot() { driveUnitLeft.writeMicroseconds(PULSE_NEUTRAL); driveUnitRight.writeMicroseconds(PULSE_RIGHT_NEUTRAL); }
void driveForward() { drive(PULSE_LEFT_CRUISE, PULSE_RIGHT_CRUISE); }
// Spin in place with the wheels moving in opposite directions. Used for maze turns, U-turns and cases 9 and 10.
void spinRight(unsigned long ms) { move(PULSE_LEFT_CREEP, PULSE_RIGHT_REVERSE, ms); }
void spinLeft(unsigned long ms) { move(PULSE_LEFT_REVERSE, PULSE_RIGHT_CREEP, ms); }
// Single-wheel pivot: only one wheel drives forward, so use twice the in-place turn duration.
void arcRight(float angle) { move(PULSE_LEFT_CREEP, PULSE_RIGHT_NEUTRAL, (unsigned long)(angle * 2 * DURATION_RIGHT_QUARTER_MS / 90.0)); }
void arcLeft(float angle) { move(PULSE_NEUTRAL, PULSE_RIGHT_CREEP, (unsigned long)(angle * 2 * DURATION_LEFT_QUARTER_MS / 90.0)); }