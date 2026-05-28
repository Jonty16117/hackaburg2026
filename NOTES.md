# HackaBurg 2026 — DuckBot

Autonomous duck that moves on water using motors, with real-time object detection.

## Concept
A motorized duck decoy / floating robot that navigates water autonomously, detecting objects in its surroundings via onboard camera + computer vision.

## Hardware

| Component | Spec |
|-----------|------|
| ESC | Bidirectional, 30A continuous |
| BEC | 5V / 2A (powers controller/receiver) |
| Battery | LiPo 2S–4S (7.4V – 16.8V) |
| Motors | 2x Brushless DC (3-wire: green, yellow, sky blue per motor) |
| Controller | TBD (Raspberry Pi / ESP32 / Arduino) |
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

## ESC Compatibility Warning

The bidirectional 30A ESC described earlier (Brushed, 2-wire output) is **not compatible** with 3-wire brushless motors. Either:

- **Option A:** Use 2x **brushless ESCs** (30A, 2S-4S, bidirectional capable) — one per motor.
- **Option B:** Replace motors with brushed DC motors to match the existing ESC.

## To-Do

- [ ] Procure duck hull / decoy
- [ ] 3D-print motor mounts
- [ ] Waterproof electronics housing
- [ ] Wire ESC + motors + battery + controller
- [ ] Calibrate motor PWM ranges
- [ ] Set up object detection pipeline (model + camera)
- [ ] Implement obstacle avoidance logic
- [ ] Field test in water
