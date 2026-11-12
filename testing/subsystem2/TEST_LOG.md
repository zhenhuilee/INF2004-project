# Subsystem 2 — test log

**Subsystem:** Producer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | stop() is accepted and completes within 200 ms | IF-06 | auto | subsystem2_test.c | [TEST] S2-U03 PASS |
| S2-U04 | idle with zero encoder errors | encoder integration | auto | subsystem2_test.c | [TEST] S2-U04 PASS |
| S2-U05 | 100 mm move finishes, both encoders count up, L/R within 10% | distance estimation | auto | subsystem2_test.c, wheels in the air | [TEST] S2-U05a/b/c PASS |
| S2-U06 | wait_done returns the error of a failed command | IF-06 | auto | subsystem2_test.c | [TEST] S2-U06 PASS | 
| S2-U07 | both motors spin forward for a positive duty | motor driver integration | manual | in subsytem2.h, S2_BRINGUP 1 and S2_BRINGUP_DUTY 350, wheels in the air | both wheels move forward for 2 sec then stop and continue again |
| S2-U08 | both encoders count while the motors run | encoder integration | manual | same build as S2-U07. Read count L= and count R= during duty=350 | both counts rise at a similar rate, stop when duty=0, err=0 | 
| S2-U09 | encoder counts per wheel turn | motion calibration | manual | in subsystem2.h, S2_BRINGUP 1, S2_BRINGUP_DUTY 0. Used post it note to mark the tyre, turn each wheel forward 10 turns by hand, read count L= and count R= | left and right within 5% |
| S2-U10 | wheel diameter | motion calibration | manual | measure across the tyre with a ruler | diameter value in cm and convert to mm |
| S2-U11 | track width | motion calibration | manual | measure from the centre of the left tyre to the centre of the right tyre with a ruler| track width value in cm and convert to mm | 
| S2-U12 | speed control holds the target speed | PID speed control, consistent speed | manual | in subsystem2.h set S2_BRINGUP 2, in task_motion_demo.c set DEMO_SPEED to 100, read tgt and spd on the lines where tgt is steady not while speeding up or slowing down| spd within 5% of tgt at each speed |
| S2-U13 | speed control holds the target speed | PID speed control, consistent speed | manual | in subsystem2.h set S2_BRINGUP 2, in task_motion_demo.c set DEMO_SPEED to 200, read tgt and spd on the lines where tgt is steady not while speeding up or slowing down| spd within 5% of tgt at each speed |
| S2-U14 | speed control holds the target speed | PID speed control, consistent speed | manual | in subsystem2.h set S2_BRINGUP 2, in task_motion_demo.c set DEMO_SPEED to 400, read tgt and spd on the lines where tgt is steady not while speeding up or slowing down| spd within 5% of tgt at each speed |
| S2-U15 | distance accuracy over 1 m | distance estimation, accurate distance | manual | in subsystem2.h S2_BRINGUP 2, in task_motion_demo.c DEMO_MODE 1. Car on the floor at a tape start line, press RST, car drives forward 1000 mm once. Measure the distance travelled with a tape measure. Repeat 5 times | mean error within 2% (980–1020 mm) |
| S2-U16 | straight-line drift over 1 m | straight-line motion correction, stable motion | manual | same runs as S2-U15. Measure how far the car ended up to the left or right of the straight line | sideways drift within 30 mm |
| S2-U17 | 360 degree turn accuracy | encoder-based turning, repeatable turns | manual | in subsystem2.h S2_BRINGUP 2, in task_motion_demo.c DEMO_MODE 2. Car on the start line, pointing along it, press RST, car turns left 360 once. Measure how far the front mark ends up left or right of the guide line, convert to degrees. Repeat 5 times | mean error within + or - 5 degrees |
| S2-U18 | 90 degree right turn accuracy | encoder-based turning, repeatable turns | manual | in subsystem2.h S2_BRINGUP 2, in task_motion_demo.c DEMO_MODE 4. Car on the guide line with its front at the crossing with the start line, press RST, car turns right 90 once. Measure how far the front ends up from the start line. Repeat 3 times. (360 degree left turns covered by S2-U17) | mean error within + or - 5 degrees |




*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-07 | S2-U07 | f1437b1 | Zhen Hui | PASS | https://youtu.be/saJUfMkZGOE | both wheels turn forward. MOTOR_RIGHT_INVERT = 1. |
| 2026-11-07 | S2-U08 | f1437b1 | Zhen Hui | FAIL | logs/2026-11-07_S2-U08_right_encoders_fail.txt | motor A worked but the right encoder for motor b have no signal as count R remained at 0. Tried to troubleshoot by using another grove port and a new grove cable. Motor b might need a replacement. Also fixed pull down bug in drv_encoder.c |
| 2026-11-10 | S2-U08 | f1437b1 | Zhen Hui | PASS | logs/2026-11-10_S2-U08_both_encoders_pass.jpeg | replaced motor b. L ≈ 361/line, R ≈ 370/line at duty 350; err = 0. B-only mode,motor a phase a gives no signal |
| 2026-11-10 | S2-U09 | f1437b1 | Zhen Hui | PASS | logs/2026-11-10_S2-U09_counts_per_turn.jpeg | left increases from 0 to 12858. 12858/10 = 1286 counts per turn, right increases from 0 to 12724. 12724/10 = 1272 counts per turn, using average = 1279 | 
| 2026-11-10 | S2-U10 | f1437b1 | Zhen Hui | PASS | ruler measurement | 5.4 cm = 54 mm |
| 2026-11-10 | S2-U11 | f1437b1 | Zhen Hui | PASS | ruler measurement | 9.6 cm = 96 mm |
| 2026-11-12 | S2-U12 | 729887e | Zhen Hui | PASS | logs/2026-11-12_S2-U12_speed_100.jpeg | 100 mm/s on the floor: spd 98–104 (max error 4%). DEMO_SPEED set to 100 |
| 2026-11-12 | S2-U13 | 729887e | Zhen Hui | PASS | logs/2026-11-12_S2-U13_speed_200.jpeg | 200 mm/s on the floor: spd 199–208 (max error 4%). DEMO_SPEED set to 200 |
| 2026-11-12 | S2-U14 | 729887e | Zhen Hui | PASS | logs/2026-11-12_S2-U14_speed_400.jpeg | 400 mm/s on the floor: spd 398–414 (max error 3.3%). DEMO_SPEED set to 400 |    
| 2026-11-12 | S2-U15 | b36cff5 | Zhen Hui | PASS | logs/2026-11-12_S2-U15_S2-U16_distance_and_straight_line.txt | 1000 mm ×5 on the floor: mean 1014.8 mm (+1.5%), range 1012–1020 mm. Wheel 62 mm, DIST_SCALE 1.06 |
| 2026-11-12 | S2-U16 | b36cff5 | Zhen Hui | PASS | logs/2026-11-12_S2-U15_S2-U16_distance_and_straight_line.txt | same 5 runs: drift 0 mm in every run |
| 2026-11-13 | S2-U17 | 2a28377 | Zhen Hui | PASS | logs/2026-11-13_S2-U17_360_turn.txt` | turn scale 1.18, 5 runs: offsets 0–5 mm right, mean error −1.4 deg, spread −2.6 to 0 deg |