# ELEC1601 Autonomous Maze Robot

University of Sydney · Introduction to Computer Systems · Group coursework

An Arduino robot uses three infrared (IR) sensors facing left, forward and right to navigate a physical maze. Our group worked on Essential and both parts of Extension.

## Arduino sketches

Open each sketch in its matching folder with the Arduino IDE and use the original board and hardware configuration.

| Stage | Sketch | What the archived code does |
| --- | --- | --- |
| Essential | [Essential.ino](essential/Essential/Essential.ino) | Uses IR readings for startup correction, corridor steering and stopping at a front wall. This is the archived sketch without the outbound/return flag. |
| Extension Part 1 | [Extension_Part_1.ino](extension/Extension_Part_1/Extension_Part_1.ino) | Uses a route flag to turn around at the first dead end and stop at the next on the return leg. |
| Extension Part 2 | [Extension_Part_2.ino](extension/Extension_Part_2/Extension_Part_2.ino) | Uses `millis()` to time navigation segments. At a three-wall dead end, it uses elapsed time to choose between turning around and stopping. |

Existing source comments are in English. The archived logic and calibration values are preserved. The sketches have not been rerun on the robot for this portfolio.

## Course requirements

The following is a summary of the supplied ELEC1601 project brief. Its distances, actions and finish conditions are assessment criteria.

### Essential

Recognise the starting scenario from the three IR sensors, show its LED pattern and execute the corresponding manoeuvre without contacting the walls. The right, middle and left indicators are connected to A0, A1 and A2 respectively.

| Scenario | Initial LEDs on or flashing | Required behaviour |
| --- | --- | --- |
| 0. Unrecognised | All off | Remain stationary. |
| 1. Centred in a corridor | Right | With similar side readings and at least 10 cm clear ahead, drive 5 cm and stop. Stay approximately parallel and within 3 cm of the corridor centreline. |
| 2–3. Right or left turn | Middle for a right turn. Right and middle for a left turn. | Detect a front wall and an open side. Turn 90° toward the opening, move into the next corridor, finish approximately parallel and within 3 cm of its centreline, then stop. |
| 4. Dead end | Left | Detect walls ahead and on both sides. Turn 180°, move forward at least 5 cm, finish approximately parallel and within 3 cm of the centreline, then stop. |
| 5–6. Parallel start near a side wall | Right and left when starting near the left wall. Middle and left when starting near the right wall. | Shift away from the near wall, recover a parallel heading and stop within 3 cm of the centreline. |
| 7–8. Angled start near a side wall | All on for a left-side start. Right flashes for a right-side start. | Recognise an approximately 30° starting angle, turn back to parallel and stop after any further correction. |
| 9–10. Team-defined scenarios | Middle flashes in one pattern. Right and middle flash in the other. | Define two further starting scenarios and their sensor checks, then distinguish the cases with their LED patterns. |

### Extension Part 1

Combine the Essential manoeuvres into four runs of increasing scope on the given maze or its mirror.

1. Recover from a challenging start, follow the first corridor and stop at the front wall.
2. Follow the first corridor, make one left or right turn, then stop at the next front wall.
3. Continue through successive corridors and turns until reaching a dead end.
4. At the dead end, turn 180° and navigate back to the start. The brief allows slight wall contact only during this turn.

### Extension Part 2

Navigate a layout that can vary between lab sessions. The maze includes an S-turn, a U-turn, a T-junction and both short and long dead ends. Escape a short dead end by pivoting or reversing without touching the walls. Finish facing forward at a long dead end, defined in the brief as a dead end after a straight section exactly two wall segments long.

The Part 2 sketch uses elapsed time as a proxy for that length. Its 5,000 ms threshold is marked as a calibration placeholder in the source.

## Attribution and distribution

Chenrui (Zach) Zhang contributed to planning, code and debugging in a group project. The group coursework is shared here for non-commercial portfolio review.

## Course reference

[University of Sydney ELEC1601 unit outline, Semester 2, 2026](https://www.sydney.edu.au/units/ELEC1601/2026-S2C-ND-CC)
