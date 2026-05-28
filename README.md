# DuckBot — HackaBurg 2026

Autonomous waterfowl robot. A motorized duck that navigates water using differential thrust and detects objects in its surroundings.

## Concept

A floating robot built into a duck decoy. Two brushless motors provide water propulsion via differential steering. Onboard Raspberry Pi runs computer vision (object detection) and ultrasonic sonar-based obstacle avoidance.

## Quick Start

See **[docs/assembly.md](docs/assembly.md)** for the full step-by-step assembly and testing guide.
See **[docs/diagrams.md](docs/diagrams.md)** for wiring diagrams and pinouts.
See **[docs/architecture.md](docs/architecture.md)** for software architecture and navigation FSM details.
See **[assembly_diagram.jpg](assembly_diagram.jpg)** for a visual wiring diagram.

```bash
# Calibrate ESCs (run once per setup)
python -m scripts.calibrate

# Test motors
python -m scripts.run

# Autonomous navigation (sense → think → act)
python -m navigation.controller
```

## Project Structure

```
hackaburg2026/
├── README.md               # You are here
├── docs/
│   ├── hardware.md          # BOM, pinouts, wiring diagrams
│   ├── architecture.md      # Software architecture, FSM, pipeline
│   ├── assembly.md           # Step-by-step assembly and testing guide
│   ├── diagrams.md           # ASCII wiring diagrams and pinouts
│   └── datasheets.md         # Component datasheets and specs
├── motors/                  # ESC + motor control
│   ├── esc.py               # Low-level ESC PWM driver
│   └── drive.py             # Differential drive (tank steer)
├── sensors/                 # Sensor drivers
│   └── sonar.py             # RCWL-1655 ultrasonic
├── navigation/              # Autonomous control
│   ├── config.py             # Tunable constants (perimeter, speeds, thresholds)
│   ├── odometry.py           # Dead reckoning from motor commands
│   ├── perimeter.py          # Virtual boundary awareness
│   ├── brain.py              # 4-state FSM with smart sonar-scan avoidance
│   └── controller.py         # Main 20 Hz sense→think→act loop
├── detection/               # Camera + object detection (future)
└── scripts/                 # Runnable entry points
    ├── calibrate.py         # ESC calibration wizard
    └── run.py               # Motor test (manual + auto sequence)
```

## Hardware Summary

| Component | Spec |
|-----------|------|
| Motors/Thrusters | 2x APISQUEEN U01 (12-16V, 2Kg thrust each, CW+CCW) |
| ESCs | 2x APISQUEEN 30A Bi-directional (BEC 5V/1A, 2-4S) |
| Servo Tester | CCPM 3-channel (MAN/NEUTRAL/AUTO) — quick test without Pi |
| Controller | Raspberry Pi |
| Sonar | RCWL-1655 ultrasonic |
| Battery | 4S1P 14.8V LiPo (2200mAh, 35C/70C, 32.56Wh) |
| Camera | TBD |
| Hull | Duck decoy / 3D printed |

## Features

- [x] Battery acquired (4S 14.8V 2200mAh 35C)
- [x] State-machine autonomous navigation (sonar + odometry + perimeter)
- [x] Forward / reverse / differential thrust motor control
- [x] ESC calibration and PWM motor control
- [x] Ultrasonic obstacle sensing (RCWL-1655)
- [ ] Field test in water
