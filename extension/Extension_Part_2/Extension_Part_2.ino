/*
 * Extension Part 2: the right wheel's neutral pulse changed from 1488 to 1495. All right-wheel pulses follow that baseline.
 * Layout: 1. Global parameters and variables. 2. Shared navigation code. 3. Part 2 timing. [P2] marks timing additions.
 * Prefer right, then straight, then left. At a three-wall dead end, stop only when elapsed time reaches LONG_DEAD_END_THRESHOLD_MS. Otherwise turn around.
 * Segment timing starts when the last turn starts, including the turn itself. Read TURN R/L/U/S and DEAD_END elapsed_ms logs at 9600 baud.
 */
#include "Servo.h"

// ======================= 1. Global parameters and variables =======================
const int PORT_DRIVE_LEFT = 13, PORT_DRIVE_RIGHT = 12;                                // Servo pins
const int PORT_EMITTER_LEFT = 10, PORT_RECEIVER_LEFT = 11, PORT_INDICATOR_LEFT = A2;  // Left IR emitter, receiver and LED
const int PORT_EMITTER_FRONT = 6, PORT_RECEIVER_FRONT = 7, PORT_INDICATOR_FRONT = A1; // Front IR
const int PORT_EMITTER_RIGHT = 2, PORT_RECEIVER_RIGHT = 3, PORT_INDICATOR_RIGHT = A0; // Right IR

// Servo pulses in microseconds: use ServoCentering to measure neutral. The reversed right wheel uses pulses below neutral to move forward.
const int PULSE_NEUTRAL = 1494, PULSE_RIGHT_NEUTRAL = 1495;  // Left and right neutral pulses (right remeasured Oct 29, previously 1488)
const int PULSE_LEFT_CRUISE = PULSE_NEUTRAL + 175, PULSE_RIGHT_CRUISE = PULSE_RIGHT_NEUTRAL - 175;  // Cruise
const int PULSE_LEFT_CREEP = PULSE_NEUTRAL + 50, PULSE_RIGHT_CREEP = PULSE_RIGHT_NEUTRAL - 50;      // Slow forward motion
const int PULSE_LEFT_REVERSE = PULSE_NEUTRAL - 50, PULSE_RIGHT_REVERSE = PULSE_RIGHT_NEUTRAL + 50;  // Reverse pulse for spinning in place
const int PULSE_LEFT_BACKWARD = PULSE_NEUTRAL - 45, PULSE_RIGHT_BACKWARD = PULSE_RIGHT_NEUTRAL + 60; // Straight reverse (increase the right pulse if the rear drifts right)
// Navigation. IR scores range from 0 to 6: 6 means no wall, lower means closer, and about 3 is centred.
const int LIMIT_PATH_CLEAR = 5, LIMIT_RIGHT_CLEAR = 6, TARGET_SIDE_SCORE = 3;  // Front or left > 5 and right >= 6 indicate clear paths. The target score is for single-wall following.
const int WAIT_RIGHT_APPROACH_MS = 820, WAIT_LEFT_APPROACH_MS = 250, DURATION_EXIT_CRUISE_MS = 500;  // Forward travel before turns and after a right turn
const int DURATION_RIGHT_QUARTER_MS = 820, DURATION_LEFT_QUARTER_MS = 820;   // Single-wheel pivot conversion for cases 5-8
// In-place maze turns of 90, 90 and 180 degrees. Decrease the time if the turn overshoots, or increase it if it falls short.
const int DURATION_MAZE_LEFT_90_MS = 660, DURATION_MAZE_RIGHT_90_MS = 1000, DURATION_MAZE_TURN_AROUND_MS = 2000;
// [P2] Long dead-end threshold in milliseconds. The value of 5000 is a placeholder for hardware calibration.
//      Use the DEAD_END logs to set it above the longest short-dead-end time and below the shortest long-dead-end time.
const unsigned long LONG_DEAD_END_THRESHOLD_MS = 5000UL;
// Cases 5 and 6: the near side scores <= 1 and the far side at least 3 higher. Cases 7 and 8 allow <= 2 because angled readings appear farther.
const int LIMIT_SIDE_NEAR = 1, LIMIT_SIDE_NEAR_ANGLED = 2, LIMIT_SIDE_DIFF = 3;
const unsigned long DISPLAY_MS = 5000;  // Show the LEDs and stay still for at least 5 seconds.
// Single-wheel angles for pivoting out and back. Reduce the first angle if it overshoots. After pivoting back, reduce the second angle if still facing the starting wall, or increase it if facing the opposite wall.
const float RECENTRE_OUT_DEG_SCENARIO5 = 30.0, RECENTRE_BACK_DEG_SCENARIO5 = 5.0;   // Case 5 (passed a hardware test)
const float RECENTRE_OUT_DEG_SCENARIO6 = 15.0, RECENTRE_BACK_DEG_SCENARIO6 = 50.0;  // Case 6
const float RECENTRE_REVERSE_CM = 8.0;       // Final reverse distance
const bool CONTINUE_AFTER_START_FIX = true;  // true continues navigation after correction. false stops for inspection.
const bool DIAGNOSTIC_LEDS = true;           // Startup diagnostics: each side LED indicates a score <= 2, and the middle LED indicates a front wall. Set false before the check.
const unsigned long DIAGNOSTIC_MS = 3000;
// Cases 9 and 10: when stuck in a front corner, a fine front score from 0 to 13 of <= 2 means the wall is very close. Increase the threshold if the corner is missed.
const int LIMIT_FRONT_CORNER_FINE = 2;
const unsigned long CORNER_DIAGONAL_BACK_MS = 600; const float CORNER_SPIN_DEG = 150.0;  // Single-wheel diagonal reverse duration and the subsequent spin angle
// Cases 7 and 8: at about 30 degrees to a wall, scan 35-41 kHz in 500 Hz steps for 20 passes (260 readings). At least 5 hits indicate a wall.
const int FRONT_EDGE_ROUNDS = 20, LIMIT_FRONT_EDGE_HITS = 5;  // Increase if cases 5 and 6 are mistaken for cases 7 and 8, or decrease for the opposite error.
const unsigned long FLASH_MS = 1000;                          // Flash 1 second on, 1 second off.
const float ALIGN_REVERSE_CM_SCENARIO7 = 6.0, ALIGN_REVERSE_CM_SCENARIO8 = 5.0;  // Reverse away from the wall first.
const float ALIGN_DEG_SCENARIO7 = 45.0, ALIGN_DEG_SCENARIO8 = 16.0;  // For the alignment pivot, increase the angle if the robot still faces the wall, or decrease it if it overshoots.
const float ALIGN_FORWARD_CM = 4.0, MS_PER_CM_CREEP = 100.0;  // Forward distance after aligning, and the time to creep forward 1 cm
const float RECENTRE_MAX_CM = 15.0, SENSOR_AHEAD_CM = 5.0;    // Maximum diagonal travel and the sensor offset ahead of the axle
// Global variables
Servo driveUnitLeft, driveUnitRight;
int scoreSideLeft = 0, scoreAhead = 0, scoreSideRight = 0;  // Current left, front and right scores
bool startChecked = false, frontWallStart = false;          // Whether the starting case was checked and whether a front wall was detected at startup (used in cases 7 and 8)
unsigned long lastTurnStartMs = 0;                          // [P2] Start time of the most recent turn
bool segmentTimerStarted = false;                           // [P2] Start timing after startup correction.

// ======================= 2. Shared navigation code (condensed) =======================
void setup() {
  pinMode(PORT_EMITTER_LEFT, OUTPUT);  pinMode(PORT_RECEIVER_LEFT, INPUT);  pinMode(PORT_INDICATOR_LEFT, OUTPUT);
  pinMode(PORT_EMITTER_FRONT, OUTPUT); pinMode(PORT_RECEIVER_FRONT, INPUT); pinMode(PORT_INDICATOR_FRONT, OUTPUT);
  pinMode(PORT_EMITTER_RIGHT, OUTPUT); pinMode(PORT_RECEIVER_RIGHT, INPUT); pinMode(PORT_INDICATOR_RIGHT, OUTPUT);
  showLeds(LOW, LOW, LOW); stopRobot();  // Leave the servos detached until the first move so the wheels stay still.
  Serial.begin(9600); Serial.print(F("Part2_Timer_v3, continue after start fix = ")); Serial.println(CONTINUE_AFTER_START_FIX ? F("yes") : F("no"));
  delay(2000);
}

void loop() {
  readAllSensors();
  Serial.print(F("L=")); Serial.print(scoreSideLeft); Serial.print(F(" M=")); Serial.print(scoreAhead); Serial.print(F(" R=")); Serial.println(scoreSideRight);

  if (!startChecked) {  // Check cases 5-10 only at startup. Evaluate each condition independently and prefer the highest matching case.
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
      case 5: showLeds(HIGH, LOW, HIGH); holdStill(DISPLAY_MS); recentre(1); finishStartFix(false); return;     // Left and right LEDs. Shift right.
      case 6: showLeds(HIGH, HIGH, LOW); holdStill(DISPLAY_MS); recentre(-1); finishStartFix(false); return;    // Left and middle LEDs. Shift left.
      case 7: showLeds(HIGH, HIGH, HIGH); holdStill(DISPLAY_MS); backTurnForward(1); finishStartFix(true); return;  // All LEDs on. Turn right until parallel.
      case 8: flashLedsStill(LOW, LOW, HIGH, DISPLAY_MS); backTurnForward(-1); finishStartFix(true); return;     // Flash the right LED. Turn left until parallel.
      case 9:  // Flash the middle LED. Reverse the left wheel to swing the rear right, then turn 150 degrees clockwise.
        flashLedsStill(LOW, HIGH, LOW, DISPLAY_MS); move(PULSE_LEFT_REVERSE, PULSE_RIGHT_NEUTRAL, CORNER_DIAGONAL_BACK_MS);
        move(PULSE_LEFT_CREEP, PULSE_RIGHT_REVERSE, (unsigned long)(CORNER_SPIN_DEG * DURATION_MAZE_RIGHT_90_MS / 90.0)); finishStartFix(false); return;
      case 10:  // Flash the middle and right LEDs. Reverse the right wheel to swing the rear left, then turn 150 degrees anticlockwise.
        flashLedsStill(LOW, HIGH, HIGH, DISPLAY_MS); move(PULSE_NEUTRAL, PULSE_RIGHT_REVERSE, CORNER_DIAGONAL_BACK_MS);
        move(PULSE_LEFT_REVERSE, PULSE_RIGHT_CREEP, (unsigned long)(CORNER_SPIN_DEG * DURATION_MAZE_LEFT_90_MS / 90.0)); finishStartFix(false); return;
      default: break;  // No startup case matched. Continue to maze navigation (cases 0-4).
    }
  }

  if (!segmentTimerStarted) beginNavigationSegment('S');  // [P2] Start timing on the first normal navigation cycle.
  // Right-hand navigation LED codes: straight = right, right turn = middle, left turn = middle and right, dead end = left.
  if (canGoLeft() && canGoForward() && canGoRight()) { showLeds(LOW, LOW, LOW); haltRobot(); }  // Case 0: all three directions are open. Stop for the rest of the run.
  else if (canGoRight()) handleRightTurn();
  else if (canGoForward()) { showLeds(LOW, LOW, HIGH); driveForward(); wallControl(); }  // Passing a left opening does not reset the timer.
  else if (canGoLeft()) handleLeftTurn();
  else handleDeadEnd();
}

// ---------- IR sensors ----------
// Take one reading at a carrier frequency. Return 0 for a reflection and 1 for no reflection.
int irDetect(int emitter, int receiver, long hz) { tone(emitter, hz); delay(1); int s = digitalRead(receiver); noTone(emitter); delay(1); return s; }
// Standard reading: six frequencies from 38 to 43 kHz, giving a score from 0 to 6.
int irDistance(int emitter, int receiver) { int missed = 0; for (long hz = 38000; hz <= 43000; hz += 1000) missed += irDetect(emitter, receiver, hz); return missed; }
void readAllSensors() {
  scoreSideLeft = irDistance(PORT_EMITTER_LEFT, PORT_RECEIVER_LEFT);
  scoreAhead = irDistance(PORT_EMITTER_FRONT, PORT_RECEIVER_FRONT);
  scoreSideRight = irDistance(PORT_EMITTER_RIGHT, PORT_RECEIVER_RIGHT);
}
bool canGoRight() { return LIMIT_RIGHT_CLEAR <= scoreSideRight; }
bool canGoForward() { return LIMIT_PATH_CLEAR < scoreAhead; }
bool canGoLeft() { return LIMIT_PATH_CLEAR < scoreSideLeft; }

// ---------- Startup case detection ----------
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
// Use the three LEDs to check cases 7 and 8 without a serial connection.
void showStartDiagnostics() {
  bool leftNear = scoreSideLeft <= LIMIT_SIDE_NEAR_ANGLED, rightNear = scoreSideRight <= LIMIT_SIDE_NEAR_ANGLED;
  Serial.print(F("Diag L/M/R = ")); Serial.print(leftNear); Serial.print(frontWallStart); Serial.println(rightNear);
  if (!DIAGNOSTIC_LEDS) return;
  showLeds(leftNear, frontWallStart, rightNear); delay(DIAGNOSTIC_MS); showLeds(LOW, LOW, LOW); delay(500);
}

// ---------- Startup manoeuvres ----------
// Cases 5 and 6: direction = 1 shifts right from the left wall, and -1 shifts left.
void recentre(int direction) {
  if (direction > 0) arcRight(RECENTRE_OUT_DEG_SCENARIO5); else arcLeft(RECENTRE_OUT_DEG_SCENARIO6);  // 1. Pivot out on one wheel.
  creepForward();  // 2. Creep diagonally until the near-side score reaches the far-side score, up to RECENTRE_MAX_CM.
  unsigned long startTimestamp = millis();
  float travelled = 0;
  bool centred = false;
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
// After startup correction, continue navigation or stop. If recheck is true after cases 7 and 8, check the offset and recentre as in cases 5 and 6.
void finishStartFix(bool recheck) { if (!CONTINUE_AFTER_START_FIX) haltRobot(); if (recheck) startChecked = false; }

// ---------- Maze manoeuvres ----------
// Cruise for ms milliseconds while reading the sensors and correcting the heading.
void cruiseFor(unsigned long ms) { unsigned long t0 = millis(); while (millis() - t0 < ms) { readAllSensors(); driveForward(); wallControl(); } }
void handleRightTurn() {  // Case 2: middle LED
  showLeds(LOW, HIGH, LOW);
  cruiseFor(WAIT_RIGHT_APPROACH_MS); stopRobot(); delay(100);  // Advance to the centre of the junction, then stop.
  beginNavigationSegment('R');                                  // [P2] Record the previous segment and start timing at this right turn.
  move(PULSE_LEFT_CREEP, PULSE_RIGHT_REVERSE, DURATION_MAZE_RIGHT_90_MS);
  cruiseFor(DURATION_EXIT_CRUISE_MS);
}
void handleLeftTurn() {   // Case 3: middle and right LEDs
  showLeds(LOW, HIGH, HIGH);
  driveForward(); delay(WAIT_LEFT_APPROACH_MS); stopRobot(); delay(100);
  beginNavigationSegment('L');                                  // [P2] As for a right turn, start timing when rotation begins.
  move(PULSE_LEFT_REVERSE, PULSE_RIGHT_CREEP, DURATION_MAZE_LEFT_90_MS);
  driveForward(); delay(200);
}
// Corridor steering (gain 30): use the side-score difference with two walls, or TARGET_SIDE_SCORE with one. Positive adj steers right and negative adj steers left.
void wallControl() {
  const float GAIN_SIDE_CONTROL = 30.0;
  int adj = 0;
  if (scoreSideLeft <= LIMIT_PATH_CLEAR && scoreSideRight <= LIMIT_PATH_CLEAR) adj = GAIN_SIDE_CONTROL * (scoreSideRight - scoreSideLeft);
  else if (scoreSideLeft < 4 && scoreSideRight > LIMIT_PATH_CLEAR) { adj = GAIN_SIDE_CONTROL * (TARGET_SIDE_SCORE - scoreSideLeft); adj = adj * 0.7; }  // Use a smaller correction with only the left wall, based on hardware tests.
  else if (scoreSideRight < 4 && scoreSideLeft > LIMIT_PATH_CLEAR) adj = -(int)(GAIN_SIDE_CONTROL * (TARGET_SIDE_SCORE - scoreSideRight));
  if (adj != 0) { driveUnitLeft.writeMicroseconds(PULSE_LEFT_CRUISE + adj); driveUnitRight.writeMicroseconds(PULSE_RIGHT_CRUISE + adj); }  // 0 means keep the cruise pulses.
}

// ---------- LEDs, stopping and basic movement ----------
void showLeds(int l, int m, int r) { digitalWrite(PORT_INDICATOR_LEFT, l); digitalWrite(PORT_INDICATOR_FRONT, m); digitalWrite(PORT_INDICATOR_RIGHT, r); }
void releaseServos() { stopRobot(); driveUnitLeft.detach(); driveUnitRight.detach(); }  // Detach the servo signals to prevent wheel creep.
void holdStill(unsigned long ms) { releaseServos(); delay(ms); }
void haltRobot() { releaseServos(); for (;;) delay(1000); }  // Stop until reset.
// Hold still for ms milliseconds and flash the selected LEDs together, 1 second on and 1 second off.
void flashLedsStill(int l, int m, int r, unsigned long ms) {
  showLeds(LOW, LOW, LOW); releaseServos();
  for (unsigned long t = 0; t < ms; t += 2 * FLASH_MS) { showLeds(l, m, r); delay(FLASH_MS); showLeds(LOW, LOW, LOW); delay(FLASH_MS); }
}
// Servos start detached and are detached while stationary. Reattach when movement is required.
void drive(int leftUs, int rightUs) {
  if (!driveUnitLeft.attached()) driveUnitLeft.attach(PORT_DRIVE_LEFT);
  if (!driveUnitRight.attached()) driveUnitRight.attach(PORT_DRIVE_RIGHT);
  driveUnitLeft.writeMicroseconds(leftUs); driveUnitRight.writeMicroseconds(rightUs);
}
// Drive with the given pulses for ms milliseconds, then stop for 0.1 seconds. This is used for turns and reversing.
void move(int leftUs, int rightUs, unsigned long ms) { drive(leftUs, rightUs); delay(ms); stopRobot(); delay(100); }
void stopRobot() { driveUnitLeft.writeMicroseconds(PULSE_NEUTRAL); driveUnitRight.writeMicroseconds(PULSE_RIGHT_NEUTRAL); }
void driveForward() { drive(PULSE_LEFT_CRUISE, PULSE_RIGHT_CRUISE); }
void creepForward() { drive(PULSE_LEFT_CREEP, PULSE_RIGHT_CREEP); }
// Single-wheel pivot: only one wheel drives forward, so use twice the in-place turn duration.
void arcRight(float angle) { move(PULSE_LEFT_CREEP, PULSE_RIGHT_NEUTRAL, (unsigned long)(angle * 2 * DURATION_RIGHT_QUARTER_MS / 90.0)); }
void arcLeft(float angle) { move(PULSE_NEUTRAL, PULSE_RIGHT_CREEP, (unsigned long)(angle * 2 * DURATION_LEFT_QUARTER_MS / 90.0)); }

// ======================= 3. [P2] Part 2 timing =======================
// Store the turn start without resetting millis(). Log the previous duration first, then update the timestamp so logging is excluded from the new segment.
void beginNavigationSegment(char turnCode) {
  unsigned long previousElapsedMs = segmentTimerStarted ? millis() - lastTurnStartMs : 0;
  Serial.print(F("TURN ")); Serial.print(turnCode); Serial.print(F(" previous_ms=")); Serial.println(previousElapsedMs);
  lastTurnStartMs = millis(); segmentTimerStarted = true;
}
// Case 4: with three walls and elapsed time at or above the threshold, stop at the long dead end. Otherwise turn 180 degrees.
void handleDeadEnd() {
  showLeds(HIGH, LOW, LOW);
  const bool threeWalls = !canGoRight() && !canGoForward() && !canGoLeft();
  const unsigned long elapsedMs = millis() - lastTurnStartMs;  // Read the elapsed time before updating the timestamp.
  const bool thresholdReached = segmentTimerStarted && elapsedMs >= LONG_DEAD_END_THRESHOLD_MS;
  stopRobot();  // Stop the wheels before logging to prevent forward drift.
  Serial.print(F("DEAD_END elapsed_ms=")); Serial.print(elapsedMs); Serial.print(F(" three_walls=")); Serial.print(threeWalls);
  Serial.print(F(" reached=")); Serial.println(thresholdReached);
  if (threeWalls && thresholdReached) { Serial.println(F("FINISH: long dead end, stopped")); haltRobot(); }  // Keep the robot facing the front wall.
  beginNavigationSegment('U');  // At a short dead end, turn around without counting encounters.
  move(PULSE_LEFT_CREEP, PULSE_RIGHT_REVERSE, DURATION_MAZE_TURN_AROUND_MS);
}
