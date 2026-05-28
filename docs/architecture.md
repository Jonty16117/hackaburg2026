# Software Architecture

## Overview

The DuckBot runs a **sense → think → act** loop on the Raspberry Pi.

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   SENSORS   │────▶│  NAVIGATION  │────▶│   MOTORS    │
│             │     │              │     │             │
│  Camera     │     │  Obstacle    │     │  Left ESC   │
│  Sonar      │     │  Avoidance   │     │  Right ESC  │
│             │     │  Path Plan   │     │             │
└─────────────┘     └──────────────┘     └─────────────┘
```

## Module Map

| Module | Purpose | Tech |
|--------|---------|------|
| `motors/esc.py` | Low-level ESC PWM driver | RPi.GPIO, 50Hz PWM |
| `motors/drive.py` | Differential drive for APISQUEEN U01 thrusters | — |
| `sensors/sonar.py` | RCWL-1655 distance readings | RPi.GPIO |
| `detection/` | Camera capture + object detection (future) | OpenCV, YOLO/TFLite |
| `navigation/` | Obstacle avoidance + path planning (future) | — |
| `scripts/` | Entry points: calibrate, run, autonomous mode | — |

## Control Flow

### Motor Control
```
Script (run.py)
    └── DuckDrive (drive.py)
            ├── Left Motor (esc.py)  → GPIO 12 → PWM → Left ESC → Left Motor
            └── Right Motor (esc.py) → GPIO 13 → PWM → Right ESC → Right Motor
```

### Sensor → Action Pipeline (future)
```
1. Camera frame captured (OpenCV)
2. Run object detection (TFLite / ONNX)
3. Sonar distance read (RCWL-1655)
4. Fuse sensor data → obstacle map
5. Compute avoidance maneuver
6. Send PWM commands to DuckDrive
```

## PWM Signal Spec

- Frequency: **50 Hz** (standard) — ESCs also support 100Hz/200Hz/500Hz
- Period: **20 ms**
- Pulse range: **1000 µs to 2000 µs**
- Neutral/stop: **1500 µs**
- Bidirectional: above 1500 = forward, below 1500 = reverse
- ESC auto-detects signal type (PWM, Oneshot, Multishot, Dshot150/300/600)

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
