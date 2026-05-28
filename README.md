# DuckBot — HackaBurg 2026

Autonomous waterfowl robot. A motorized duck that navigates water using differential thrust and detects objects in its surroundings.

## Concept

A floating robot built into a duck decoy. Two brushless motors provide water propulsion via differential steering. Onboard Raspberry Pi runs computer vision (object detection) and ultrasonic sonar-based obstacle avoidance.

## Quick Start

```bash
# Calibrate ESCs (run once per setup)
python -m scripts.calibrate

# Test motors
python -m scripts.run
```

## Project Structure

```
hackaburg2026/
├── README.md               # You are here
├── docs/
│   ├── hardware.md          # BOM, pinouts, wiring diagrams
│   └── architecture.md      # Software architecture & data flow
├── motors/                  # ESC + motor control
│   ├── esc.py               # Low-level ESC PWM driver
│   └── drive.py             # Differential drive (tank steer)
├── sensors/                 # Sensor drivers
│   └── sonar.py             # RCWL-1655 ultrasonic
├── detection/               # Camera + object detection (future)
├── navigation/              # Obstacle avoidance + autonomy (future)
└── scripts/                 # Runnable entry points
    ├── calibrate.py         # ESC calibration wizard
    └── run.py               # Motor test (manual + auto sequence)
```

## Hardware Summary

| Component | Spec |
|-----------|------|
| Motors | 2x Brushless DC (3-wire, 2S-4S) |
| ESCs | 2x Bidirectional BL ESC 30A (BEC 5V/2A) |
| Controller | Raspberry Pi |
| Sonar | RCWL-1655 ultrasonic |
| Battery | LiPo 2S-4S (TBD) |
| Camera | TBD |
| Hull | Duck decoy / 3D printed |

## Features

- [ ] Forward / reverse / turning via differential thrust
- [ ] ESC calibration and PWM motor control
- [ ] Ultrasonic obstacle sensing (RCWL-1655)
- [ ] Real-time camera object detection
- [ ] Autonomous obstacle avoidance
- [ ] Waterproof electronics enclosure
- [ ] Field test in water
