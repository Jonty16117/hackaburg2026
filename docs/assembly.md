# Assembly Guide — Motor Test Setup

## Parts Checklist

| # | Part | Notes |
|---|------|-------|
| 2 | APISQUEEN U01 Thrusters | One CW, one CCW (counter-rotating pair) |
| 2 | APISQUEEN 30A Bi-directional ESC | 2-4S, BEC 5V/1A; 30A > 17A per thruster |
| 1 | Raspberry Pi | Any model with 40-pin GPIO |
| 1 | CCPM Servo Tester | 3-channel, 3 modes: MAN / NEUTRAL / AUTO |
| 1 | LiPo Battery 4S1P 14.8V | 2200mAh, 35C/70C, 32.56Wh — onboard |
| 1 | Power bank / USB-C cable | Powers the Pi |
| - | Jumper wires (female-female) | For ESC signal → Pi GPIO |
| - | Screwdriver / wire strippers | For any terminal blocks |
| - | Electrical tape / heat shrink | Insulate connections |

### About CW/CCW Thruster Pair

The U01 comes in CW (clockwise) and CCW (counter-clockwise) versions. For differential steering, use one of each — this naturally cancels out torque and lets both thrusters push in the same direction when mounted opposite. Mount them so both face the same way on the hull.

## Quick Test with Servo Tester (No Pi Required)

This is the fastest way to verify your motors and ESCs work. Nothing but battery + ESC + servo tester.

### Wiring

```
    LiPo Battery (3S-4S)
          │
    ┌─────┴─────┐
    │           │
    │ B+    B-  │
    │ ●     ●   │
    │  ESC      │
    │           │
    │ ●  ●  ●   │──── 3 black wires ──── U01 Thruster
    │           │
    │ Br R  O   │── 3-pin signal header
    └──┬──┬──┬──┘
       │  │  │
       │  │  └── Orange ── CH1 signal (yellow/white on servo tester)
       │  └───── Red ───── CH1 +5V (powered by ESC BEC)
       └──────── Brown ─── CH1 GND
           │
    ┌──────┴──────┐
    │   CH1  CH2  CH3  │◄── Only need CH1
    │  [S]  [±]  [S]  [±]  [S]  [±]  │
    │   Servo Tester     │
    │  MAN  NEUTRAL  AUTO│
    │        [KNOB]      │
    └────────────────────┘
```

### Steps

1. **Wire ESC → motor:** ESC 3 black wires → U01 thruster (green, yellow, sky blue).
2. **Wire ESC → servo tester:** Plug ESC 3-pin header (brown/red/orange) into CH1 on the servo tester. Match polarity — brown=GND, red=5V, orange=signal.
3. **Connect battery:** Thick red (+) and black (-) from ESC to LiPo.
4. **Switch to MAN mode:** Turn knob to minimum (counter-clockwise).
5. **Plug in battery:** ESC will beep — it's alive.
6. **Slowly turn knob clockwise:** Motor should spin up. Turn CCW to spin down.
7. **Switch to AUTO mode:** Motor sweeps automatically full range.
8. **Switch to NEUTRAL:** Motor stops (1500µs center).

Repeat for the second ESC/thruster on CH2 or CH3.

> The servo tester is powered by the ESC's BEC (5V) via the red wire — no separate power needed.

---

## Pi-Based Setup & Testing

If you want software control (differential steering, object detection, etc.), continue below.

## Before You Start

- **WARNING:** Brushless motors can spin at high RPM. Remove propellers before first power-up. Secure motors to a surface.
- **WARNING:** LiPo batteries can catch fire if shorted. Never leave battery connected unattended.
- Do NOT connect ESC BEC 5V (red signal wire) to the Pi — the Pi is USB-powered separately.
- Keep ESCs dry during testing (unless you have the waterproof version).

---

## Wiring Overview

```
                    LiPo Battery (2S-4S)
                        │
          ┌─────────────┼─────────────┐
          │             │             │
    ┌─────┴─────┐ ┌─────┴─────┐       │
    │ Left ESC  │ │ Right ESC │       │
    │           │ │           │       │
    │ B+ B-     │ │ B+ B-     │       │
    │ •  •      │ │ •  •      │       │
    └──┬──┬──┬──┘ └──┬──┬──┬──┘       │
       │  │  │       │  │  │          │
       │  │  │       │  │  │          │
    ┌──┘  │  └──┐ ┌──┘  │  └──┐       │
    │     │     │ │     │     │       │
    G  Y  SB    G  Y  SB              │
  Left Motor   Right Motor            │
                                      │
       ┌──────────────────────────────┘
       │  (Signal side — 3-pin header)
       │
  ┌────┴────┐  ┌──────────┐
  │ Left ESC │  │ Right ESC│
  │ Br R  O  │  │ Br R  O  │
  └──┬───┬───┘  └──┬───┬───┘
     │   │         │   │
     │   │ RED:    │   │ RED:
     │   │ DO NOT  │   │ DO NOT
     │   │ CONNECT │   │ CONNECT
     │   │         │   │
     └───┼── GPIO 12   │
         │             │
         └─── Pi GND ──┘
```

---

## Step 1: Wire Motors to ESCs

Each ESC has 3 black wires on one end. Connect them to the 3 motor wires.

| ESC Black Wire | Left Motor Wire | Right Motor Wire |
|:---:|:---:|:---:|
| Black 1 | Green | Green |
| Black 2 | Yellow | Yellow |
| Black 3 | Sky Blue | Sky Blue |

The order isn't critical — if a motor spins the wrong direction later, swap any two wires to reverse it.

**Secure connections** with electrical tape or bullet connectors. In water, these must be fully waterproofed (epoxy potting or silicone sealant).

---

## Step 2: Wire ESC Signal to Raspberry Pi

Each ESC has a 3-pin signal header on the power side: **Brown (GND), Red (BEC 5V), Orange (PWM)**.

| ESC Wire | Pi Pin (BCM) | Pi Pin (Board) |
|----------|:------------:|:--------------:|
| Left Brown (GND) | GND | Pin 6 or 9 |
| Left Orange (PWM) | GPIO 12 | Pin 32 |
| Left Red (5V) | **DO NOT CONNECT** | — |
| Right Brown (GND) | GND | Pin 14 or 20 |
| Right Orange (PWM) | GPIO 13 | Pin 33 |
| Right Red (5V) | **DO NOT CONNECT** | — |

Use female-female jumper wires.

> The Red wire outputs 5V from the ESC's BEC to power a receiver. The Pi is already powered via USB. Connecting both power sources to the same rail creates a backfeed risk.

---

## Step 3: Battery Wiring

The thick **Red** wire from each ESC goes to LiPo **positive (+)**.
The thick **Black** wire from each ESC goes to LiPo **negative (-)**.

Both ESCs connect to the same battery. Use a parallel connector (XT60 Y-harness) or solder both leads together.

**Do NOT connect the battery yet.** Keep it disconnected until you are ready to calibrate.

---

## Step 4: Power Up the Pi

1. Ensure all signal/ground wires are correctly seated on GPIO pins.
2. Double-check NO red BEC wire is connected.
3. Power the Pi via USB from a power bank or wall adapter.
4. SSH into the Pi or open a terminal.

Verify GPIO is accessible:
```bash
python3 -c "import RPi.GPIO; print('OK')"
```

---

## Step 5: ESC Calibration

Calibration tells the ESC what 100% and 0% throttle look like. Run once per setup.

```bash
cd ~/hackaburg2026
python -m scripts.calibrate
```

The script will guide you:

1. **DISCONNECT battery** — make sure no power is going to ESCs.
2. Press Enter — the script sends a **2000µs (full forward)** pulse.
3. **CONNECT battery** — the ESCs will beep once acknowledging max throttle.
4. Press Enter — the script sends a **1000µs (full reverse/min)** pulse.
5. ESCs beep again confirming the range.
6. Script sends **1500µs (neutral)** — ESCs are armed and ready.

---

## Step 6: Run Motor Test

```bash
python -m scripts.run
```

Choose manual (`m`) or auto (`a`) mode.

**Manual controls:**
| Key | Action |
|:---:|--------|
| `w` | Forward |
| `s` | Reverse |
| `a` | Turn left |
| `d` | Turn right |
| `q` | Stop |
| `1-9` | Set speed (1=10%, 9=90%) |
| `x` | Exit |

**What to check during the first test:**
- Both motors spin when commanded.
- Forward means both spin the same direction.
- If one motor spins opposite, swap any two of its 3 motor wires.
- No smoke, no excessive heat from ESC or motor.
- Motors respond smoothly to speed changes.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Motor doesn't spin | ESC not calibrated | Re-run `python -m scripts.calibrate` |
| Motor twitches/vibrates | Bad phase connection | Check all 3 motor-ESC wire joints |
| Motor spins wrong direction | Phase order | Swap any 2 of the 3 motor wires |
| ESC hot, motor stuttering | Over-current | Check KV/prop size against 30A rating |
| No beep from ESC at power-on | No signal / no battery | Check GPIO connections, battery voltage |
| `ImportError: No module named 'RPi'` | Missing library | `pip install RPi.GPIO` |
| `RuntimeError: Not running on a Raspberry Pi` | Developing on laptop | Transfer code to Pi, or mock GPIO for testing |

---

## Next Steps

- [x] Acquire LiPo battery — **4S 14.8V 2200mAh on hand**
- [ ] Source or 3D-print motor mounts for U01 (75×75mm footprint, STP file available from APISQUEEN)
- [ ] Waterproof all connections (epoxy potting or marine heat shrink)
- [ ] Mount thrusters facing same direction (CW on one side, CCW on other)
- [ ] Integrate RCWL-1655 sonar
- [ ] Integrate camera for object detection
