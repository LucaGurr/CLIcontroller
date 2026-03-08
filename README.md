# CLIcontroller

A CLI robot arm controller for a 2-link planar arm with base rotation, written in plain C (standard library + math). It solves inverse kinematics to convert Cartesian target coordinates into joint angles, then drives 4 stepper motors through A4988 drivers via GPIO. A Docker Compose setup provides a simulated GPIO environment with a real-time pin activity monitor, so you can develop and test without physical hardware.

## Why Inverse Kinematics?

When you want a robot arm to reach a point in space, the natural way to think about it is in Cartesian coordinates — "go to position (x, y, z)". But motors don't understand coordinates; they understand angles. Something needs to translate between the two, and that something is inverse kinematics.

The alternative — forward kinematics — works the other way around: given joint angles, compute where the end effector ends up. That's mathematically straightforward but operationally useless for control, because it forces you to guess angles and hope the arm lands where you want. Inverse kinematics lets you specify *where* and have the math figure out *how*.

For a 2-link planar arm this has a clean closed-form solution (no iterative solvers, no Jacobians, no numerical approximations). That makes it fast, deterministic, and easy to implement in pure C without pulling in any linear algebra libraries. It's the right level of complexity for a real physical arm — the math is non-trivial enough to be interesting but tractable enough to run on a microcontroller or SBC without any overhead.

## The Math

The arm is modeled as two rigid links of lengths **L1** (lower/shoulder, default 500 mm) and **L2** (upper/elbow, default 525 mm) rotating in a vertical plane (x-z), mounted on a base that rotates around the vertical axis (y).

Given a target point `(x, y_rot, z)` where `x` and `z` are millimeters in the vertical plane and `y_rot` is the base rotation in degrees:

### Reachability check

First, compute the planar distance from the shoulder to the target:

```
dist = sqrt(x² + z²)
```

The target is reachable only if `|L1 - L2| ≤ dist ≤ L1 + L2`. If it falls outside this annular workspace, the program reports an out-of-range error.

### Elbow angle (q2) — Law of Cosines

The triangle formed by L1, L2, and the line from shoulder to target gives:

```
cos(q2) = (dist² - L1² - L2²) / (2 · L1 · L2)
q2 = acos(cos(q2))
```

The `cos(q2)` value is clamped to `[-1, 1]` before calling `acos` to guard against floating-point drift at the workspace boundary.

### Shoulder angle (q1) — atan2 decomposition

The shoulder angle is the angle to the target minus the angular offset introduced by the elbow bend:

```
q1 = atan2(z, x) - atan2(L2 · sin(q2), L1 + L2 · cos(q2))
```

Using `atan2` instead of `atan` handles all four quadrants correctly without special-casing.

### Gearbox scaling

The shoulder and elbow joints use a 16:1 gearbox, so the computed angles are multiplied by 16 to get the motor-side angle. The base has no gearbox (1:1).

### Angle-to-steps conversion

The A4988 drivers operate in full-step mode by default (1.8° per step, 200 steps per revolution). The conversion from degrees to step pulses is:

```
steps = |angle_deg| × 200 / 360
```

For the shoulder and elbow, `angle_deg` is the already-scaled motor-side angle (joint angle × 16), so no additional gearbox math is needed at the step-generation stage. For the base, `angle_deg` is the raw joint angle.

The sign of the angle determines the direction pin state: positive angles set DIR high (CW), negative angles set DIR low (CCW). Each step is a single LOW→HIGH→LOW pulse on the STEP pin, with `STEP_DELAY_US` (15000 µs) between each transition. That gives a stepping rate of ~33 steps/second, or roughly 10 RPM at the motor shaft.

## Hardware: GPIO Pin Mapping

The arm uses 4 A4988 stepper driver boards, each requiring exactly 2 GPIO pins: one for STEP (pulse to advance one step) and one for DIR (direction). All 8 pins are directly wired to a Raspberry Pi GPIO header:

```
A4988 Driver   Signal   Raspberry Pi GPIO   Joint
────────────   ──────   ─────────────────   ──────────────
A4988 #1       STEP     GPIO 17             Motor 1 (Base)
A4988 #1       DIR      GPIO 27             Motor 1 (Base)
A4988 #2       STEP     GPIO 22             Motor 2 (Shoulder)
A4988 #2       DIR      GPIO 23             Motor 2 (Shoulder)
A4988 #3       STEP     GPIO 24             Motor 3 (Elbow)
A4988 #3       DIR      GPIO 25             Motor 3 (Elbow)
A4988 #4       STEP     GPIO 5              Motor 4 (End Effector)
A4988 #4       DIR      GPIO 6              Motor 4 (End Effector)
```

These are defined as `#define` constants in `control.c` (`MOTOR1_STEP_PIN`, `MOTOR1_DIR_PIN`, etc.) and also stored in the `motorPins[4][2]` array for indexed access.

## GPIO Driver Layer

The program talks to GPIO through the Linux sysfs interface. The path prefix is controlled by the `GPIO_BASE_PATH` macro, which defaults to `/sys/class/gpio` (the real Pi path). For the Docker simulation it is overridden at compile time with `-DGPIO_BASE_PATH='"/gpio"'`.

The driver layer consists of six functions:

- **`gpio_export(pin)`** — Writes the pin number to `GPIO_BASE_PATH/export` to request the kernel (or simulator) to create the `gpioN/` directory. Skips if the directory already exists (checked via `access()`).
- **`gpio_unexport(pin)`** — Writes the pin number to `GPIO_BASE_PATH/unexport` to release the pin back to the kernel after use.
- **`gpio_set_direction(pin, dir)`** — Writes `"out"` or `"in"` to `gpioN/direction`.
- **`gpio_write(pin, value)`** — Writes `0` or `1` to `gpioN/value`. This is the function called hundreds of times per motor movement to pulse STEP pins.
- **`gpio_read(pin)`** — Reads and returns the current value (`0` or `1`) from `gpioN/value`. Returns `-1` on error.
- **`log_activity(fmt, ...)`** — A `printf`-style variadic function that appends a line to `GPIO_BASE_PATH/activity.log`. The GPIO monitor container tails this file in real time.

The init/cleanup lifecycle:

1. **`gpio_init()`** — Called at the start of `passAnglesToDriver()`. Exports all 8 pins, sets them all to output mode, and writes 0 (LOW) to every pin.
2. **`step_motor(motor_id, step_pin, dir_pin, angle_deg)`** — Sets the DIR pin based on angle sign, then pulses the STEP pin `steps` times with `STEP_DELAY_US` between each transition.
3. **`gpio_cleanup()`** — Writes 0 to all 8 pins and unexports each one after all motors have finished moving.

Every init, drive, completion, and cleanup event is logged to `activity.log` with a structured prefix (`INIT:`, `DRIVE:`, `DONE:`, `CLEANUP:`) that the monitor parses for color-coded display.

## Simultaneous Motor Drive

All three motors (base, shoulder, elbow) move at the same time. Before executing, the program generates a repeating tick pattern and displays it for confirmation.

### Pattern Generation

The algorithm computes the smallest repeating section that, when tiled, produces exactly the required step count for each motor.

**Step 1 — Compute GCD and section parameters:**

```
g            = gcd(steps_m1, steps_m2, steps_m3)
section_len  = max_steps / g
repetitions  = g
```

For example, with steps `600 / 300 / 100`:

```
g = gcd(600, 300, 100) = 100
section_len = 600 / 100 = 6
repetitions = 100
```

**Step 2 — Distribute each motor's steps across the section (Bresenham):**

Each motor gets `total_steps / g` steps spread evenly across `section_len` ticks using the Bresenham line algorithm:

```
error = section_len / 2
for each tick:
    error -= steps_in_section
    if error < 0:
        pulse this tick  (green block)
        error += section_len
    else:
        skip this tick   (red block)
```

The motor with the most steps always fills the entire section (steps every tick). Slower motors step at evenly distributed intervals. If the step counts share no common factor (e.g. primes like `7 / 5 / 3`), `g = 1`, so `section_len = max_steps` and the pattern runs exactly once.

Continuing the 600/300/100 example:

```
Motor 1 (Base):      ██████  6 of 6 ticks  ×  100 reps  =  600 steps
Motor 2 (Shoulder):  ░█░█░█  3 of 6 ticks  ×  100 reps  =  300 steps
Motor 3 (Elbow):     ░░░█░░  1 of 6 ticks  ×  100 reps  =  100 steps
```

**Step 3 — Verification:**

After generation the program counts steps in each pattern and multiplies by repetitions. If any count doesn't match the target, the pattern is regenerated with a different Bresenham starting error. Three offsets are tried (`section_len/2`, `0`, `section_len-1`) before aborting with an error.

**Step 4 — Confirmation display:**

The section is printed with ANSI colored full-block characters: green (█) for a step tick, red (█) for a skipped tick. If `section_len` exceeds 60, the display is truncated. Section length, repetition count, and per-motor step totals are shown. The user confirms before execution starts.

**Step 5 — Execution:**

Direction pins for all motors are set first, then the nested loop runs:

```
for each repetition:
    for each tick in section:
        HIGH on all step_pins that step this tick  (simultaneously)
        wait STEP_DELAY_US
        LOW  on all step_pins that step this tick  (simultaneously)
        wait STEP_DELAY_US
```

This is implemented in `driveMotorsSimultaneous()`, called from `passAnglesToDriver()`. The function takes arrays of step counts, directions, step pins, and dir pins, so it is not hardcoded to a specific number of motors.

## Self-test

The main menu offers **[3] Self-test**, which runs a battery of regression tests on the inverse kinematics and tick-pattern logic without driving any hardware. All tests use known-good inputs and expected outputs; the suite is designed so that any change to the IK formula, angle-to-step conversion, or Bresenham pattern generation will cause one or more tests to fail, making it safe to refactor or port the code.

### Why these tests exist

- **Correctness:** The IK solution and step patterns are derived from closed-form math and integer arithmetic. A single sign error, wrong constant, or off-by-one in the GCD/Bresenham path would produce wrong angles or wrong step totals. The selftest locks in the current, verified behavior.
- **Edge cases:** The formulas have natural boundaries (reachability, `acos` domain, `atan2(0,0)`, division by GCD, all-zero steps). Those boundaries are where bugs typically appear (e.g. out-of-range, undefined behavior, or wrong rejection). The tests explicitly hit those boundaries so regressions are caught immediately.
- **Determinism:** Expected values were obtained by running the same formulas (by hand or from the code) once and recording the results. The test then compares the running code’s output to those values. A small tolerance (±20 motor degrees) is allowed only where integer truncation in the IK can change the last digit; all other tests require an exact match.

### How the edge cases were chosen

- **Reachability:** The condition `|L1 − L2| ≤ dist ≤ L1 + L2` defines an annulus. We test:
  - **Inside:** Points clearly inside (e.g. full stretch, 90° elbow, min reach folded, horizontal 725 mm, base angle 90°) to ensure the solver returns consistent angles.
  - **Outside:** Points just outside (too far, origin, just beyond max, just inside min) to ensure the solver correctly reports “out of reach” and does not return angles.
  - **Boundary:** Full stretch and min reach sit on the boundary (`dist = L1 + L2` and `dist = |L1 − L2|`); these stress `acos` clamping and the exact comparison.
- **Quadrants and geometry:** We test positive and negative x (full stretch +x / −x), positive and negative z (straight up, straight down), and mixed x/z (e.g. 500,525 and 725,0) so that all quadrants and the `atan2(z, x)` behavior are exercised.
- **Degenerate geometry:** When L1 = L2, the inner radius is zero; the “origin” target (0,0) lies on that circle. The triangle degenerates (both `atan2` terms can be `atan2(0,0)`, which is implementation-defined in C). We test both “equal segments full stretch” and “equal segments origin (folded)” so that L1 = L2 and the (0,0) case are covered; the expected shoulder angle for (0,0) is taken from the actual implementation (e.g. −1440 motor degrees) because the standard does not fix `atan2(0,0)`.
- **Tick patterns:** The Bresenham path is sensitive to:
  - **All zeros:** `max_steps == 0`; no allocation, no division by GCD. Ensures we don’t dereference or free uninitialized pointers.
  - **Single motor:** Only one motor has steps; others are zero. GCD and section length must still be correct.
  - **Equal steps:** All motors the same; section length 1, reps = that step count. Catches GCD and “one tick per rep” behavior.
  - **Coprime steps:** e.g. (7, 11, 13); GCD = 1, section length = max_steps. Stress-tests the Bresenham distribution over a long section.
  - **Common GCD:** e.g. (16, 32, 48), (64, 128, 192), (3, 6, 9). Ensures `section_len = max_steps / g` and `reps = g` are correct and that per-section step counts multiply back to the requested totals.
  - **Minimal and single-step:** (1,1,1) and (1,0,0), (0,0,1), etc. Ensure small counts and “only one motor steps” don’t break the algorithm.
  - **Large counts:** e.g. (1000, 500, 250). Ensures no overflow or obvious scaling errors.
- **End-to-end consistency:** One test converts fixed IK-style angles to steps, runs `buildPatterns`, and checks that for each motor the sum of steps in the pattern × repetitions equals the requested step count. That ties IK → steps → pattern together.

### Inverse kinematics test cases (L1 = 500 mm, L2 = 525 mm unless noted)

| Test | Target (x, base, z) | Why it’s an edge case | Expected (base, shoulder_motor, elbow_motor) |
|------|--------------------|------------------------|---------------------------------------------|
| Full stretch +x | (1025, 0, 0) | Boundary: dist = L1 + L2; cos(q2) = 1, q2 = 0; arm straight. | (0, 0, 0) |
| Full stretch −x | (−1025, 0, 0) | Same boundary, negative x; exercises atan2(z,x) in second quadrant. | (0, 2880, 0) |
| Straight up (z only) | (0, 0, 1025) | Boundary, z-only; atan2(1025, 0) = π/2. | (0, 1440, 0) |
| Straight down | (0, 0, −1025) | Boundary, negative z; atan2(−1025, 0) = −π/2. | (0, −1440, 0) |
| 90° elbow at (500, 525) | (500, 0, 525) | dist² = L1² + L2² ⇒ cos(q2) = 0, q2 = 90°; nice closed form. | (0, 0, 1440) |
| Min reach folded (25, 0) | (25, 0, 0) | Boundary: dist = L1 − L2; cos(q2) = −1, q2 = 180°; arm fully folded. Shoulder can truncate to −179° → tolerance. | (0, −2880, 2880) ± tolerance |
| Base angle 90° | (1025, 90, 0) | Base is passed through; ensures base is not mixed into planar IK. | (90, 0, 0) |
| Horizontal 725 mm | (725, 0, 0) | Elbow 90°, shoulder not zero; mixed geometry. | (0, −736, 1440) |
| Out of reach (too far) | (1500, 0, 0) | dist > L1 + L2 ⇒ must reject. | reject |
| Out of reach (origin) | (0, 0, 0) with L1≠L2 | dist = 0 < \|L1−L2\| ⇒ must reject. | reject |
| Out of reach (just beyond max) | (1026, 0, 0) | dist = 1026 > 1025 ⇒ reject. | reject |
| Out of reach (just inside min) | (24, 0, 0) | dist = 24 < 25 ⇒ reject. | reject |
| Equal segments L1=L2=400 full stretch | (800, 0, 0) | L1 = L2; inner radius 0; boundary dist = 2L. | (0, 0, 0) |
| Equal segments origin (folded, L1=L2) | (0, 0, 0) with L1=L2=400 | dist = 0 = \|L1−L2\|; degenerate atan2(0,0). Expected angle from implementation. | (0, −1440, 2880) |
| L1 > L2 full stretch | (1025, 0, 0), L1=525, L2=500 | Formula is symmetric in L1/L2; ensures no hidden L1&lt;L2 assumption. | (0, 0, 0) |

### Tick-pattern test cases

| Test | Step counts | Why it’s an edge case |
|------|-------------|----------------------|
| All zeros | (0, 0, 0) | max_steps = 0; no allocation; must not free uninitialized pointers. |
| Single motor | (100,0,0), (0,100,0), (0,0,100) | Only one motor steps; GCD and section_len must still be correct for that motor. |
| Equal | (100,100,100), (200,200,200), (17,17,17) | section_len = 1, reps = step count; every tick steps all motors. |
| Coprime | (7, 11, 13) | GCD = 1 ⇒ section_len = 13, reps = 1; long section, no repetition. |
| GCD &gt; 1 | (16,32,48), (64,128,192), (3,6,9) | section_len = max/g, reps = g; verifies integer division and step totals. |
| Minimal | (1,1,1), (1,0,0), (0,0,1) | Smallest non-zero counts; single-step on one motor. |
| Large | (1000, 500, 250) | Larger numbers; no overflow, correct scaling. |

### IK → steps → pattern consistency

One test takes fixed motor angles (e.g. base 0°, shoulder 90° motor, elbow 45° motor), converts to steps, runs `buildPatterns`, and asserts that for each motor the sum of pattern entries × repetitions equals the requested step count. This ensures the full pipeline (angles → steps → Bresenham → execution totals) is consistent.

### Running the self-test

From the main menu, choose **[3] Self-test**. The program prints each test name and **[PASS]** or **[FAIL]** with details on failure, then a summary (e.g. “All 31 tests passed” or “2 failed, 29 passed”). Press any key to return to the main menu. No GPIO is touched.

## Build & Run

### Native (no GPIO simulation)

No build system — compile directly with gcc:

```bash
gcc -o control control.c -lm
./control
```

`-lm` is required for math functions (`sqrt`, `acos`, `atan2`, `sin`, `cos`, `fabs`). This uses the default `GPIO_BASE_PATH` of `/sys/class/gpio`, which is the correct path on a Raspberry Pi. When run on a machine without a GPIO sysfs tree, all GPIO writes silently fail but all IK math and menu navigation works normally.

To build with the Docker simulation path instead:

```bash
gcc -o control control.c -lm -DGPIO_BASE_PATH='"/gpio"'
```

### Docker with GPIO Simulation

The Docker Compose setup runs two containers sharing a named volume (`gpio_data`) mounted at `/gpio` in both:

**Prerequisites:** Docker and Docker Compose.

**Step 1 — Build both images:**

```bash
docker compose build
```

This builds:
- `clicontroller-gpio-sim` — Alpine 3.19 with bash and coreutils. Runs `setup.sh` (creates the fake sysfs directory tree) followed by `monitor.sh` (tails the activity log with color-coded output).
- `clicontroller-control` — Alpine 3.19 with gcc and musl-dev. Compiles `control.c` at image build time. The entrypoint (`entrypoint.sh`) polls for `/gpio/gpio17/value` to exist before launching the binary, ensuring the simulator is ready.

**Step 2 — Start the GPIO simulator and pin monitor (Terminal 1):**

```bash
docker compose up gpio-sim
```

This does the following in order:
1. Creates the `/gpio/` directory in the shared volume.
2. Creates `/gpio/export` and `/gpio/unexport` files (present for sysfs API compatibility, but functionally inert in simulation).
3. For each of the 8 pins (17, 27, 22, 23, 24, 25, 5, 6), creates `/gpio/gpioN/direction` (initially `"in"`) and `/gpio/gpioN/value` (initially `"0"`).
4. Creates `/gpio/activity.log` (empty).
5. Launches the monitor, which prints the pin mapping table and then runs `tail -f` on `activity.log`, parsing each line and printing it with a timestamp and color/icon based on the prefix:
   - `INIT:` — blue, ⚡ icon
   - `DRIVE:` — yellow, ⟳ icon (shows motor name, direction, step count, angle)
   - `DONE:` — green, ✓ icon
   - `CLEANUP:` — red, ■ icon, followed by a dump of all 8 pin value/direction states

**Step 3 — Run the controller interactively (Terminal 2):**

```bash
docker compose run control
```

This starts an interactive session. Navigate the menus, enter coordinates, and confirm the IK result. When you choose "pass angles to Driver", the program will:
1. Export and configure all 8 GPIO pins (writes appear in Terminal 1 as `INIT:` lines).
2. Sequentially drive each motor — set the DIR pin, then pulse the STEP pin for the calculated number of steps. Each motor appears in Terminal 1 as a `DRIVE:` line followed by a `DONE:` line.
3. Reset all pins to LOW (`CLEANUP:` in Terminal 1), followed by a full pin state dump.

**Step 4 — Teardown:**

```bash
docker compose down -v
```

The `-v` flag removes the `gpio_data` volume. Without it, stale GPIO files and logs persist across runs.

### Docker Compose Architecture

```
┌─────────────────────────────────┐     ┌─────────────────────────────────┐
│  gpio-sim container             │     │  control container              │
│  (Alpine + bash + coreutils)    │     │  (Alpine + gcc + musl-dev)      │
│                                 │     │                                 │
│  setup.sh                       │     │  entrypoint.sh                  │
│    creates /gpio/gpio{N}/       │     │    polls for /gpio/gpio17/value │
│    creates /gpio/activity.log   │     │    then exec ./control          │
│                                 │     │                                 │
│  monitor.sh                     │     │  control (compiled C binary)    │
│    tail -f /gpio/activity.log   │     │    writes /gpio/gpio{N}/value   │
│    color-coded event display    │     │    appends /gpio/activity.log   │
│                                 │     │                                 │
│  /gpio  ◄──── gpio_data ────►  /gpio  │
│         (shared Docker volume)          │
└─────────────────────────────────┘     └─────────────────────────────────┘
```

The `gpio-sim` service has a healthcheck (`test -f /gpio/gpio17/value`, polled every 500ms, up to 10 retries) and the `control` service uses `depends_on: condition: service_healthy`, so the control container only starts after the simulated sysfs tree exists.

### File-Level Detail of the Simulated sysfs Tree

After `setup.sh` runs, the shared volume contains:

```
/gpio/
├── export              # writable file (A4988 compat, functionally inert)
├── unexport            # writable file (A4988 compat, functionally inert)
├── activity.log        # appended to by control.c, tailed by monitor.sh
├── gpio17/
│   ├── direction       # "in" initially, set to "out" by gpio_init()
│   └── value           # "0" initially, pulsed 0→1→0 by step_motor()
├── gpio27/
│   ├── direction
│   └── value
├── gpio22/
│   ├── direction
│   └── value
├── gpio23/
│   ├── direction
│   └── value
├── gpio24/
│   ├── direction
│   └── value
├── gpio25/
│   ├── direction
│   └── value
├── gpio5/
│   ├── direction
│   └── value
└── gpio6/
    ├── direction
    └── value
```

Each `value` file is opened, written (`"0"` or `"1"`), and closed on every single step pulse. For a 45° base rotation that's 25 step pulses × 2 writes per pulse (HIGH then LOW) = 50 file writes just for that one motor. The shoulder and elbow joints, after 16× gearbox scaling, will have far more writes — a 45° shoulder joint angle becomes 720° motor-side, which is 400 steps = 800 file writes.

## Program Flow

The program is a menu-driven TUI using ANSI escape codes for color and screen clearing. Headings and key values (coordinates, angles, step counts, etc.) are highlighted in baby blue (RGB 137,207,240) for readability.

```
main
 └── menuStructure                   (Main Menu: IK, Raw, Self-test, Exit)
      ├── inverseKinematicsMenu      (choose default or custom arm lengths)
           ├── getCoords              (enter initial coordinates x, y, z)
           │    └── getTargetCoords   (enter target coordinates x, y, z)
           │         └── InverseKinematicsCalculations
           │              ├── [OOR] → getTargetCoords  (if unreachable)
           │              └── [OK]  → passAnglesToDriver
           │                   ├── gpio_init()        (export + configure 8 pins)
           │                   ├── step_motor() ×3    (base, shoulder, elbow)
           │                   ├── gpio_cleanup()     (all pins LOW)
           │                   └── menuStructure       (back to main menu)
           └── customSecLenMenu      (enter L1, L2 in mm → getCoords)
      ├── rawMovementMenu             (raw steps or angles → tick pattern → driver)
      └── run_selftest                (IK + tick-pattern regression tests; no GPIO)
```

Menus navigate by recursive calls. Invalid or out-of-range input returns the user to the appropriate previous menu.

## Project Structure

```
CLIcontroller/
├── control.c            # entire application — IK, menus, GPIO driver, stepper control
├── Dockerfile           # Alpine-based build for the control binary
├── entrypoint.sh        # waits for GPIO sim readiness, then exec ./control
├── docker-compose.yml   # two-service setup: gpio-sim + control, shared volume
├── gpio-sim/
│   ├── Dockerfile       # Alpine + bash + coreutils
│   ├── setup.sh         # creates fake sysfs directory tree, then exec monitor.sh
│   └── monitor.sh       # tails activity.log with color-coded formatted output
├── AGENTS.md            # AI coding assistant context
└── README.md            # this file
```