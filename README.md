# CLIcontroller

A CLI robot arm controller for a 2-link planar arm with base rotation, written in plain C (standard library + math). It solves inverse kinematics to convert Cartesian target coordinates into joint angles, and will eventually drive stepper motors via A4988 drivers over GPIO.

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

## Build & Run

No build system yet — compile directly with gcc:

```bash
gcc -o control control.c -lm
./control
```

`-lm` is required for math functions (`sqrt`, `acos`, `atan2`, `sin`, `cos`, `fabs`).

## Program Flow

The program is a menu-driven TUI using ANSI escape codes for color and screen clearing:

```
main → menuStructure → inverseKinematicsMenu → getCoords (initial)
  → getTargetCoords → InverseKinematicsCalculations → passAnglesToDriver
```

Menus navigate by recursive calls. Invalid or out-of-range input returns the user to the appropriate previous menu.

## Future Plans

The current implementation computes joint angles and prints them. The next major milestone is actually driving real hardware. This is tracked across several GitHub issues:

- **GPIO Hardware Abstraction** ([#1](https://github.com/LucaGurr/CLIcontroller/issues/1)) — A swappable GPIO layer (real hardware via `libgpiod` or simulation via stdout) with platform detection, so the code runs on any Linux box during development and on a target SBC for deployment.

- **A4988 Stepper Driver Interface** ([#2](https://github.com/LucaGurr/CLIcontroller/issues/2)) — The core hardware interface. Pin assignments stored as a 2D array `pinout[4][2]` (step pin, dir pin per motor). Functions for initializing drivers, pulsing steps with correct A4988 timing (≥1 µs pulse width, ≥200 ns DIR setup), and optional ENABLE/SLEEP pin support.

- **Angle-to-Steps Conversion** ([#3](https://github.com/LucaGurr/CLIcontroller/issues/3)) — Translating the IK output (degrees) into discrete step counts for the A4988. This involves per-motor config (steps/rev, microstepping mode, gear ratio) and tracking current position in steps for delta computation.

- **Motion Control & Trajectory** ([#4](https://github.com/LucaGurr/CLIcontroller/issues/4)) — Moving beyond "teleport to position": acceleration ramps (trapezoidal), coordinated multi-axis movement so all joints finish simultaneously, speed control, and eventually a homing sequence.

- **Safety & Signal Handling** ([#5](https://github.com/LucaGurr/CLIcontroller/issues/5)) — Software joint limits, SIGINT/SIGTERM handlers for clean GPIO shutdown, emergency stop support, and endstop inputs.

- **Build System & Config** ([#6](https://github.com/LucaGurr/CLIcontroller/issues/6)) — A Makefile with `-DSIMULATION` flag, pin configuration headers (`pinout.h`, `motor_config.h`), and eventually splitting the single-file architecture into separate modules.
