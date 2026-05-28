# HackaBurg 2026 — DuckBot

Autonomous duck that moves on water using motors, with real-time object detection.

## Concept
A motorized duck decoy / floating robot that navigates water autonomously, detecting objects in its surroundings via onboard camera + computer vision.

## Hardware

| Component | Spec |
|-----------|------|
| ESCs | 2x Bidirectional BL ESC, 30A (3 black → motor; red/black → battery; brown/red/orange → signal) |
| BEC | 5V / 2A built into ESCs |
| Battery | LiPo 2S–4S (7.4V – 16.8V) — not yet acquired |
| Motors | 2x Brushless DC (3-wire: green, yellow, sky blue per motor) |
| Controller | Raspberry Pi (GPIO PWM → ESC signal wires) |
| Sonar | RCWL-1655 (ultrasonic distance sensor for obstacle detection) |
| Camera | TBD (for object detection) |
| Hull | 3D-printed / modified duck decoy |

## Software

| Component | Details |
|-----------|---------|
| Object Detection | YOLOv8 / MobileNet SSD (on-device inference) |
| Motor Control | PWM over ESC via GPIO |
| Navigation | Rule-based or simple obstacle avoidance |

## Features

- [ ] Forward / reverse / turning via differential thrust
- [ ] Real-time object detection on camera feed
- [ ] Obstacle avoidance
- [ ] Waterproof electronics enclosure
- [ ] Telemetry / remote override (optional)

## Architecture

```
Camera → Object Detection → Navigation Logic → ESC/Motors
                        ↓
                   Telemetry (optional)
```

## Motor Wiring

Each motor has 3 phase wires (no polarity — order determines direction):

| Wire Color | Connection |
|------------|------------|
| Green | BL ESC Phase A / U |
| Yellow | BL ESC Phase B / V |
| Sky Blue | BL ESC Phase C / W |

- Swap any **two wires** to reverse rotation direction.
- Both motors wired identically = both spin same way (counter-rotating if props are mirrored).

## ESC Pinout

Each ESC has two sides:

| Side | Wires | Connects To |
|------|-------|-------------|
| Motor side | 3x Black | Motor phases (green, yellow, sky blue) |
| Power side | Red (thick) + Black (thick) | LiPo battery |
| Signal | 3-pin header: Brown / Red / Orange | Raspberry Pi |

Signal pinout:

| Wire | Function | Connect to Pi |
|------|----------|---------------|
| Brown | GND | Pi GND (pin 6, 9, 14, etc.) |
| Red | BEC 5V output | **Do NOT connect** (Pi powered via USB) |
| Orange | PWM signal | Pi GPIO 12 (left ESC), GPIO 13 (right ESC) |

### Differential Steering (Tank Drive)

| Action | Left Motor | Right Motor |
|--------|------------|-------------|
| Forward | Full CW | Full CW |
| Reverse | Full CCW | Full CCW |
| Left Turn | Stop/Rev | Forward |
| Right Turn | Forward | Stop/Rev |
| Stop | 1500µs neutral | 1500µs neutral |

## To-Do

- [ ] Procure duck hull / decoy
- [ ] 3D-print motor mounts
- [ ] Waterproof electronics housing
- [x] Wire ESC + motors + battery + controller
- [ ] Calibrate motor PWM ranges
- [ ] Set up object detection pipeline (model + camera)
- [ ] Implement obstacle avoidance logic
- [ ] Field test in water
