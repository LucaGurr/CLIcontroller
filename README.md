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

The sign of the angle determines the direction pin state: positive angles set DIR high (CW), negative angles set DIR low (CCW). Each step is a single LOW→HIGH→LOW pulse on the STEP pin, with `STEP_DELAY_US` (1500 µs) between each transition. That gives a stepping rate of ~333 steps/second, or roughly 6 RPM at the motor shaft.

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

The program talks to GPIO through the Linux sysfs interface (`/gpio/` in Docker, `/sys/class/gpio/` on real hardware). The path prefix is controlled by the `GPIO_BASE_PATH` macro. The driver layer consists of four functions:

- **`gpio_export(pin)`** — Writes the pin number to `GPIO_BASE_PATH/export` to request the kernel (or simulator) to create the `gpioN/` directory. Skips if the directory already exists (checked via `access()`).
- **`gpio_set_direction(pin, dir)`** — Writes `"out"` or `"in"` to `gpioN/direction`.
- **`gpio_write(pin, value)`** — Writes `0` or `1` to `gpioN/value`. This is the function called hundreds of times per motor movement to pulse STEP pins.
- **`log_activity(fmt, ...)`** — A `printf`-style variadic function that appends a line to `GPIO_BASE_PATH/activity.log`. The GPIO monitor container tails this file in real time.

The init/cleanup lifecycle:

1. **`gpio_init()`** — Called at the start of `passAnglesToDriver()`. Exports all 8 pins, sets them all to output mode, and writes 0 (LOW) to every pin.
2. **`step_motor(motor_id, step_pin, dir_pin, angle_deg)`** — Sets the DIR pin based on angle sign, then pulses the STEP pin `steps` times with `STEP_DELAY_US` between each transition.
3. **`gpio_cleanup()`** — Writes 0 to all 8 pins after all motors have finished moving.

Every init, drive, completion, and cleanup event is logged to `activity.log` with a structured prefix (`INIT:`, `DRIVE:`, `DONE:`, `CLEANUP:`) that the monitor parses for color-coded display.

## Build & Run

### Native (no GPIO simulation)

No build system — compile directly with gcc:

```bash
gcc -o control control.c -lm
./control
```

`-lm` is required for math functions (`sqrt`, `acos`, `atan2`, `sin`, `cos`, `fabs`). When run natively, GPIO writes will silently fail (no `/gpio/` directory exists), but all IK math and menu navigation works normally.

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

The program is a menu-driven TUI using ANSI escape codes for color and screen clearing:

```
main
 └── menuStructure                   (Main Menu: IK mode or exit)
      └── inverseKinematicsMenu      (choose default or custom arm lengths)
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

## Future Plans

The current implementation computes joint angles and drives simulated GPIO pins. The next major milestone is running on real hardware. This is tracked across several GitHub issues:

- **GPIO Hardware Abstraction** ([#1](https://github.com/LucaGurr/CLIcontroller/issues/1)) — A swappable GPIO layer (real hardware via `libgpiod` or simulation via sysfs files) with platform detection, so the code runs on any Linux box during development and on a target SBC for deployment.

- **A4988 Stepper Driver Interface** ([#2](https://github.com/LucaGurr/CLIcontroller/issues/2)) — The core hardware interface. Pin assignments stored as a 2D array `pinout[4][2]` (step pin, dir pin per motor). Functions for initializing drivers, pulsing steps with correct A4988 timing (≥1 µs pulse width, ≥200 ns DIR setup), and optional ENABLE/SLEEP pin support.

- **Angle-to-Steps Conversion** ([#3](https://github.com/LucaGurr/CLIcontroller/issues/3)) — Translating the IK output (degrees) into discrete step counts for the A4988. This involves per-motor config (steps/rev, microstepping mode, gear ratio) and tracking current position in steps for delta computation.

- **Motion Control & Trajectory** ([#4](https://github.com/LucaGurr/CLIcontroller/issues/4)) — Moving beyond "teleport to position": acceleration ramps (trapezoidal), coordinated multi-axis movement so all joints finish simultaneously, speed control, and eventually a homing sequence.

- **Safety & Signal Handling** ([#5](https://github.com/LucaGurr/CLIcontroller/issues/5)) — Software joint limits, SIGINT/SIGTERM handlers for clean GPIO shutdown, emergency stop support, and endstop inputs.

- **Build System & Config** ([#6](https://github.com/LucaGurr/CLIcontroller/issues/6)) — A Makefile with `-DSIMULATION` flag, pin configuration headers (`pinout.h`, `motor_config.h`), and eventually splitting the single-file architecture into separate modules.
