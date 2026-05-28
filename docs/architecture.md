# Software Architecture

## Overview

The DuckBot runs a **sense → think → act** loop on the Raspberry Pi at ~20 Hz.

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   SENSORS   │────▶│  NAVIGATION  │────▶│   MOTORS    │
│             │     │              │     │             │
│  Sonar      │     │  Odometry    │     │  Left ESC   │
│  (RCWL-1655)│     │  Perimeter   │     │  Right ESC  │
│             │     │  Brain FSM   │     │  (PWM 50Hz) │
└─────────────┘     └──────────────┘     └─────────────┘
      ▲                                        │
      └────────────────────────────────────────┘
               feedback loop
```

## Module Map

| Module | Purpose | Hardware Dep |
|--------|---------|-------------|
| `motors/esc.py` | Low-level ESC PWM driver (1000–2000µs at 50Hz) | RPi.GPIO |
| `motors/drive.py` | Differential drive + motor inversion + `drive_speeds()` | esc.py |
| `sensors/sonar.py` | RCWL-1655 ultrasonic distance readings | RPi.GPIO |
| `navigation/config.py` | All tunable constants in one place | — |
| `navigation/odometry.py` | Dead reckoning: integrates speed×dt → (x, y, θ) pose | — |
| `navigation/perimeter.py` | Virtual boundary: point-in-rectangle, distance-to-edge, bearing-to-center | — |
| `navigation/brain.py` | 4-state FSM: EXPLORE → AVOID → TURN_TO_CENTER → STUCK | perimeter.py |
| `navigation/controller.py` | Main ~20 Hz sense→think→act loop + CLI entry point | all above |
| `detection/` | Camera capture + object detection (future) | OpenCV, TFLite |
| `scripts/` | Entry points: calibrate.py, run.py, generate_diagram.py | — |

## Control Flow

### Manual Motor Control

```
Script (run.py)
    └── DuckDrive (drive.py)
            ├── forward() / reverse() / turn_left() / turn_right()
            │   └── _invert() → maps robot-centric speeds through motor wiring
            ├── drive_speeds(left, right)  ← used by autonomous controller
            └── Left/Right Motor (esc.py)
                └── GPIO 12/13 → 50Hz PWM → ESCs → Thrusters
```

### Autonomous Navigation Pipeline

Implemented in `navigation/controller.py`:

```
┌──────────────────────────────────────────────────────────────┐
│                  CONTROLLER LOOP (~20 Hz)                     │
│                                                              │
│  1. odom.update(last_left, last_right, dt)    # integrate    │
│  2. distance = sonar.distance_cm()              # sense       │
│  3. left, right = brain.decide(distance, x,    # decide      │
│                                y, θ, dt)                     │
│  4. drive.drive_speeds(left, right)             # act        │
└──────────────────────────────────────────────────────────────┘
```

## State Machine (brain.py)

```
                     ┌──────────┐
                ┌───▶│ EXPLORE  │
                │    │          │
                │    │ fwd 40%  │
  cooldown      │    │ + jitter │
  expired       │    └────┬─────┘
                │         │
                │    ┌────┴────────────┐
                │    │                 │
                │    │ sonar < 50cm    │ near perimeter
                │    ▼                 ▼
                │  ┌──────────────┐  ┌──────────────────┐
                │  │    AVOID     │  │ TURN_TO_CENTER   │
                │  │              │  │                  │
                │  │ 1. rev 0.5s  │  │ turn toward      │
                │  │ 2. scan L30° │  │ centroid, then   │
                │  │ 3. read sonar│  │ drive forward    │
                │  │ 4. scan R30° │  │ until clear      │
                │  │ 5. read sonar│  └────────┬─────────┘
                │  │ 6. pick dir  │           │
                │  │ 7. turn ~70° │  aligned  │
                │  └──────┬───────┘           │
                │         │                   │
                │         │  3× in 10s        │
                │         ▼                   │
                │  ┌──────────────┐           │
                │  │    STUCK     │           │
                │  │ rev + tight  │           │
                │  │ turn 2s      │───────┐   │
                │  └──────────────┘       │   │
                │                         │   │
                └─────────────────────────┴───┘
```

### AVOID Scan Sequence

When an obstacle triggers AVOID, the robot does a smart look-left-look-right scan to pick the clearer escape direction:

| Time | Phase | Motors (L,R) | Sonar |
|------|-------|-------------|-------|
| 0.0s | REVERSE | (-0.5, -0.5) | — |
| 0.5s | SCAN_LEFT (~30°) | (-0.4, +0.4) | — |
| 1.1s | READ_LEFT (pause) | (0.0, 0.0) | d_left |
| 1.25s | SCAN_RIGHT (~60°) | (+0.4, -0.4) | — |
| 2.45s | READ_RIGHT (pause) | (0.0, 0.0) | d_right |
| 2.6s | COMPLETE_TURN (~70°) | depends on pick | — |
| 3.4s | → EXPLORE | (+0.4, +0.4) | — |

Direction choice: pick the side with the larger sonar reading (more clearance). If one reading is invalid, pick the valid side. If both are invalid, alternate.

### Perimeter Handling (TURN_TO_CENTER)

```
perimeter.check:
  ├── is_inside(x, y) == False  →  force TURN_TO_CENTER
  └── distance_to_edge < 30cm   →  force TURN_TO_CENTER

TURN_TO_CENTER behavior:
  ├── heading already toward center  →  drive forward until edge clears
  └── heading not toward center      →  turn in place (shorter arc)
```

### STUCK Fallback

If AVOID triggers 3 times within a 10-second window, the robot enters STUCK — a longer escape maneuver (reverse + tight turn for 2 seconds). This breaks oscillation in tight corners.

## Dead Reckoning (Odometry)

Position is estimated from motor speed commands using a differential drive model:

```
v     = (v_left + v_right) / 2           linear velocity (cm/s)
ω     = (v_right - v_left) / wheel_base  angular velocity (rad/s)

x     += v · cos(θ) · dt
y     += v · sin(θ) · dt
θ     += ω · dt                          (normalized to [-π, π])
```

Where `v_left = left_speed * MAX_SPEED_CM_S` and `v_right = right_speed * MAX_SPEED_CM_S`.

> Accuracy depends on **`WHEEL_BASE_CM`** and **`MAX_SPEED_CM_S`** being tuned in water. Odometry drifts over time — the perimeter margin provides a safety buffer. These values are in `navigation/config.py`.

## PWM Signal Spec

- Frequency: **50 Hz** (standard) — ESCs also support 100Hz/200Hz/500Hz
- Period: **20 ms**
- Pulse range: **1000 µs to 2000 µs**
- Neutral/stop: **1500 µs**
- Bidirectional: above 1500 = forward, below 1500 = reverse
- Speed-to-PWM: `pulse = 1500 + clamp(speed, -1.0, +1.0) * 500`

## Motor Specs (APISQUEEN U01)

| Spec | Value |
|------|-------|
| Voltage | 12-16V (3S-4S LiPo) |
| Max power | 390W per thruster |
| Max current | 17A per thruster |
| Thrust | 2Kg per thruster |
| Size | 75×75mm |
| Weight | 178g |
| CW/CCW | Available in both — use one of each for counter-rotating pair |

## GPIO Allocation

| BCM Pin | Function |
|:-------:|----------|
| 12 | Left ESC PWM signal |
| 13 | Right ESC PWM signal |
| 23 | RCWL-1655 TRIG |
| 24 | RCWL-1655 ECHO |
| 17, 27, 22 | Reserved (status LEDs, etc.) |
