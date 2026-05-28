# Hardware Reference

## Bill of Materials (BOM)

| # | Component | Spec | Notes |
|---|-----------|------|-------|
| 2 | APISQUEEN U01 Thrusters | 12-16V (3S-4S), 390W, 17A max, 2Kg thrust | 75×75mm, 178g, freshwater, CW+CCW pair |
| 2 | APISQUEEN 30A Bi-directional ESC | 2-4S LiPo, BEC 5V/1A, 14AWG | 28×15×6mm, 36g, 30A > 17A max per thruster |
| - | APISQUEEN — [apisqueen.net](https://apisqueen.net) | Underwater thruster/motor/ESC manufacturer | Shop: [underwaterthruster.com](https://www.underwaterthruster.com) |
| 1 | Raspberry Pi | Any with GPIO (Pi 3/4/5/Zero 2W) | Runs motor control + detection |
| 1 | RCWL-1655 | Ultrasonic distance sensor | Object avoidance |
| 1 | CCPM Servo Tester | 3-channel, knob control, 1000-2000µs PWM output | Quick motor test without Pi |
| 1 | LiPo Battery | 4S1P 14.8V, 2200mAh, 35C/70C burst, 32.56Wh | 77A cont / 154A burst, plenty for 2 thrusters (34A max combined) |
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
| Thin header | Red | BEC +5V / 1A output | ⚠️ Leave disconnected (Pi powered via USB) |
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

---

## Battery Runtime Estimates

Battery: 4S1P 14.8V, 2200mAh, 32.56Wh

| Throttle | Current (per thruster) | Total draw | Runtime |
|----------|:---------------------:|:----------:|---------|
| Full (100%) | 17A | 34A | ~4 min |
| Half (50%) | ~8.5A | ~17A | ~8 min |
| Cruise (30%) | ~5A | ~10A | ~13 min |
| Idle | 0A | 0.2A (ESCs) | hours |

> 4S (16.8V fully charged) is the U01's **optimal voltage** — max thrust and efficiency.
> Battery burst rating 154A >> 34A max draw — no sag under load.

---

## Navigation Controller

The autonomous control loop runs at ~20 Hz in `navigation/controller.py`:

```
Sonar (RCWL-1655) ──┐
                    ├──▶ Brain FSM ──▶ DuckDrive ──▶ ESCs → Thrusters
Odometry (x,y,θ) ───┘       ▲
                          │
Perimeter (10m×2m) ───────┘
```

### Tunable Parameters (in `navigation/config.py`)

| Parameter | Default | Purpose | Calibration |
|-----------|---------|---------|-------------|
| `MAX_SPEED_CM_S` | 100 | cm/s at full throttle (speed=1.0) | Time 2m sprint in water |
| `WHEEL_BASE_CM` | 30 | Distance between thruster centers | Measure with ruler |
| `OBSTACLE_THRESHOLD_CM` | 50 | Sonar distance that triggers AVOID | Test with obstacle |
| `PERIMETER_MARGIN_CM` | 30 | How close to edge before turning | Larger = safer, less coverage |
| `SONAR_TRIG` / `SONAR_ECHO` | GPIO 23 / 24 | RCWL-1655 pin assignment | Match physical wiring |
| `LEFT_PIN` / `RIGHT_PIN` | GPIO 12 / 13 | ESC PWM signal pins | Match physical wiring |

### Perimeter

Defined as a rectangle in `navigation/config.py`:

```python
PERIMETER_WIDTH_CM  = 1000  # 10 meters
PERIMETER_HEIGHT_CM = 200   # 2 meters
START_X_CM = 500   # center X
START_Y_CM = 100   # center Y
START_HEADING_RAD = 0.0  # facing +X (East)
```

The robot starts at center pointing along the long axis. Odometry tracks position from motor commands. When within `PERIMETER_MARGIN_CM` of any edge, the robot turns toward center and drives inward.

### Operational States

| State | Trigger | Action |
|-------|---------|--------|
| **EXPLORE** | Default | Forward at 40% speed with periodic jitter |
| **AVOID** | Sonar < 50cm | Reverse → scan L/R → turn toward clearer side |
| **TURN_TO_CENTER** | Near perimeter edge | Turn toward centroid → drive forward until clear |
| **STUCK** | 3× AVOID in 10s | Reverse + tight turn escape for 2s |
