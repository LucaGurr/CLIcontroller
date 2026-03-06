# AGENTS.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

A CLI robot arm controller written in plain C (standard library + math). It computes inverse kinematics for a 2-link planar arm and (in the future) passes joint angles to a GPIO driver for 8 pins. The entire application lives in a single file: `control.c`.

## Build and Run

There is no Makefile or build system. Compile directly with gcc. The `-lm` flag is required for math functions (`sqrt`, `acos`, `atan2`, `sin`, `cos`, `fabs`).

```
gcc -o control control.c -lm
./control
```

There are no tests or linting tools configured.

## Architecture

The program is a menu-driven TUI using ANSI escape codes for color and screen clearing.

**Flow:** `main` → `menuStructure` → `inverseKinematicsMenu` → `getCoords` (initial) → `getTargetCoords` → `InverseKinematicsCalculations` → `passAnglesToDriver`

**Key globals:**
- `lengths[2]` — arm segment lengths in mm (L1=500, L2=525 default)
- `angles[4]` — computed joint angles: base rotation, shoulder, elbow, end effector (indices 0–3)
- `targetcoords[3]` / `initcoords[3]` — target and initial coordinates as (x mm, y degrees base rotation, z mm)

**IK math:** Standard 2-link inverse kinematics using the law of cosines for elbow angle (q2) and atan2 for shoulder angle (q1). Shoulder and elbow angles are scaled by 16× for a 16:1 gearbox ratio. Base rotation is passed through directly.

**Navigation pattern:** Menus call each other recursively (no loop). Invalid/out-of-range input returns the user to the previous menu via recursive call.

## Conventions

- ANSI color macros (`RED`, `BLUE`, `GREEN`, `YELLOW`, `RESET`) are used for all terminal output formatting.
- Screen clearing uses raw ANSI escape sequences via `clearScreen()`.
- `getchar(); getchar();` is used as a "press any key" pause pattern (consuming the leftover newline from prior `scanf`).
