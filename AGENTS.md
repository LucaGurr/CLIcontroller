# AGENTS.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

A CLI robot arm controller written in plain C (standard library + math). It computes inverse kinematics for a 2-link planar arm and drives 4 A4988 stepper motors via GPIO sysfs. A Docker Compose setup simulates the GPIO pins with a real-time monitor. The core application lives in a single file: `control.c`.

## Build and Run

### Native

No Makefile or build system. Compile directly with gcc. The `-lm` flag is required for math functions (`sqrt`, `acos`, `atan2`, `sin`, `cos`, `fabs`).

```
gcc -o control control.c -lm
./control
```

GPIO writes silently fail when run natively (no `/gpio/` directory), but all menus and IK math work.

### Docker (with GPIO simulation)

```
docker compose build
docker compose up gpio-sim          # Terminal 1: pin monitor
docker compose run control           # Terminal 2: interactive controller
docker compose down -v               # cleanup (removes gpio_data volume)
```

Two containers share a `gpio_data` volume mounted at `/gpio`. The `gpio-sim` container creates a fake sysfs directory tree and monitors `activity.log`. The `control` container writes pin states and appends log entries.

There are no tests or linting tools configured.

## Architecture

The program is a menu-driven TUI using ANSI escape codes for color and screen clearing.

**Flow:** `main` → `menuStructure` → `inverseKinematicsMenu` → `getCoords` (initial) → `getTargetCoords` → `InverseKinematicsCalculations` → `passAnglesToDriver` → `gpio_init` → `step_motor` ×3 → `gpio_cleanup`

**Key globals:**
- `lengths[2]` — arm segment lengths in mm (L1=500, L2=525 default)
- `angles[4]` — computed joint angles: base rotation, shoulder, elbow, end effector (indices 0–3)
- `targetcoords[3]` / `initcoords[3]` — target and initial coordinates as (x mm, y degrees base rotation, z mm)
- `motorPins[4][2]` — GPIO pin numbers per motor: `{step_pin, dir_pin}`
- `ALL_PINS[8]` — flat list of all GPIO pin numbers for bulk init/cleanup

**GPIO pin mapping (A4988 → Raspberry Pi GPIO):**
- Motor 1 (Base): STEP=17, DIR=27
- Motor 2 (Shoulder): STEP=22, DIR=23
- Motor 3 (Elbow): STEP=24, DIR=25
- Motor 4 (End Effector): STEP=5, DIR=6

**GPIO interface:** Functions `gpio_export()`, `gpio_set_direction()`, `gpio_write()` operate on sysfs files under `GPIO_BASE_PATH` (default `/gpio`). `log_activity()` is a variadic `printf`-style function that appends to `activity.log`. `step_motor()` sets the DIR pin then pulses STEP with `STEP_DELAY_US` (1500 µs) between transitions. Steps = `|angle| × 200 / 360`.

**IK math:** Standard 2-link inverse kinematics using the law of cosines for elbow angle (q2) and atan2 for shoulder angle (q1). Shoulder and elbow angles are scaled by 16× for a 16:1 gearbox ratio. Base rotation is passed through directly.

**Navigation pattern:** Menus call each other recursively (no loop). Invalid/out-of-range input returns the user to the previous menu via recursive call.

## File Structure

- `control.c` — entire application: IK, menus, GPIO driver, stepper control
- `Dockerfile` — Alpine-based image that compiles control.c
- `entrypoint.sh` — polls for GPIO sim readiness, then exec ./control
- `docker-compose.yml` — two services (gpio-sim, control) with shared volume
- `gpio-sim/Dockerfile` — Alpine + bash + coreutils
- `gpio-sim/setup.sh` — creates fake sysfs tree under /gpio, then exec monitor.sh
- `gpio-sim/monitor.sh` — tails activity.log with color-coded output

## Conventions

- ANSI color macros (`RED`, `BLUE`, `GREEN`, `YELLOW`, `RESET`) are used for all terminal output formatting.
- Screen clearing uses raw ANSI escape sequences via `clearScreen()`.
- `getchar(); getchar();` is used as a "press any key" pause pattern (consuming the leftover newline from prior `scanf`).
- GPIO pin numbers are defined as `#define` constants (`MOTOR1_STEP_PIN`, etc.) and mirrored in `motorPins[4][2]`.
- Activity log lines use structured prefixes: `INIT:`, `DRIVE:`, `DONE:`, `CLEANUP:` — parsed by the monitor for formatting.
