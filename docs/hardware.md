# Hardware Reference

## Bill of Materials (BOM)

| # | Component | Spec | Notes |
|---|-----------|------|-------|
| 2 | Brushless DC Motors | 3-wire (green, yellow, sky blue) | Counter-rotating pair |
| 2 | Bidirectional BL ESC | 30A, BEC 5V/2A, 2S-4S LiPo | 3 black → motor; thick red/black → battery; 3-pin signal header |
| 1 | Raspberry Pi | Any with GPIO (Pi 3/4/5/Zero 2W) | Runs motor control + detection |
| 1 | RCWL-1655 | Ultrasonic distance sensor | Obiect avoidance |
| 1 | LiPo Battery | 2S-4S (7.4V – 16.8V) | Powers ESCs → motors; BEC powers Pi (optional) |
| 1 | Camera | USB or CSI | For object detection |
| 1 | Duck Decoy Hull | Commercial / 3D-printed | Houses all electronics |

---

## ESC Pinout

Each ESC has two ends:

### Motor Side (3 black wires)
Connect to the 3 motor phase wires. Order determines direction — swap any two to reverse.

| ESC Black Wire | Motor Wire | Phase |
|:---:|:---:|:---:|
| Black 1 | Green | U / A |
| Black 2 | Yellow | V / B |
| Black 3 | Sky Blue | W / C |

### Power Side

| Wire | Color | Function | Connect To |
|------|-------|----------|------------|
| Thick | Red | Battery + | LiPo positive (+) |
| Thick | Black | Battery - | LiPo negative (-) |
| Thin header | Brown | GND | Raspberry Pi GND (e.g., pin 6) |
| Thin header | Red | BEC +5V output | ⚠️ Leave disconnected (Pi powered via USB) |
| Thin header | Orange | PWM signal | Raspberry Pi GPIO |

---

## Raspberry Pi GPIO Wiring

| Pi Pin (BCM) | Pi Pin (Board) | Connected To |
|:---:|:---:|------|
| 12 | 32 | Left ESC signal (orange) |
| 13 | 33 | Right ESC signal (orange) |
| GND | 6, 9, 14, 20, 25, 30, 34, 39 | Both ESC GND (brown) |
| 23 | 16 | RCWL-1655 TRIG |
| 24 | 18 | RCWL-1655 ECHO |

### RCWL-1655 Sensor

| RCWL Pin | Connect To |
|----------|------------|
| VCC (5V) | Pi 5V (pin 2 or 4) |
| GND | Pi GND |
| TRIG | Pi GPIO 23 |
| ECHO | Pi GPIO 24 (via voltage divider: 4.7kΩ + 10kΩ to drop 5V → 3.3V) |

---

## Power Architecture

```
LiPo Battery (2S-4S, 7.4-16.8V)
    │
    ├── Left ESC  ──→ Left Motor (3-phase)
    │       └── BEC 5V/2A (not used)
    │
    └── Right ESC ──→ Right Motor (3-phase)
            └── BEC 5V/2A (not used)

Raspberry Pi ── USB power bank / separate 5V supply
```

⚠️ Do NOT connect ESC BEC 5V output to Raspberry Pi while Pi is USB-powered — this creates a ground loop / backfeed risk.

---

## Differential Steering (Tank Drive)

| Action | Left Motor | Right Motor | PWM (µs) Left | PWM (µs) Right |
|--------|:----------:|:-----------:|:-------------:|:--------------:|
| Stop | — | — | 1500 (neutral) | 1500 (neutral) |
| Forward | CW (forward) | CW (forward) | 1501 → 2000 | 1501 → 2000 |
| Reverse | CCW (reverse) | CCW (reverse) | 1499 → 1000 | 1499 → 1000 |
| Turn Left | CCW (reverse) | CW (forward) | 1499 → 1000 | 1501 → 2000 |
| Turn Right | CW (forward) | CCW (reverse) | 1501 → 2000 | 1499 → 1000 |

### Pulse Width Ranges

| Pulse (µs) | Throttle |
|-------------|----------|
| 1000 | Full reverse |
| 1500 | Neutral / stop |
| 2000 | Full forward |
