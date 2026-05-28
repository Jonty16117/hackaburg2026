# Wiring Diagrams

## Full System

```
                                ┌─────────────────────┐
                                │   Raspberry Pi       │
                                │   (USB powered)      │
                                │                      │
                                │  GPIO12  GPIO13      │
                                │    │        │        │
                                │    │        │        │
                                │   GND  ────GND───────┼───────────────────┐
                                └────┼────────┼────────┘                   │
                                     │        │                            │
                               ORANGE│   ORANGE│                           │
                                     │        │                            │
    ┌────────────────────────────┐   │   ┌────┴────────────────────────┐   │
    │        LEFT ESC            │   │   │        RIGHT ESC            │   │
    │                            │   │   │                            │   │
    │  ┌─── brown  ──────────────┼───┘   │  ┌─── brown  ──────────────┼───┘
    │  │   red    ── NOT CONNECTED│       │  │   red    ── NOT CONNECTED│
    │  │── orange ── GPIO12 ─────┼───────┼──│── orange ── GPIO13      │
    │  │                          │       │  │                          │
    │  │   B+ ────┐               │       │  │   B+ ────┐               │
    │  │   B- ──┐  │              │       │  │   B- ──┐  │              │
    │  └────────┼──┼──────────────┘       │  └────────┼──┼──────────────┘
    │           │  │                      │           │  │
    │  3 black  │  │                      │  3 black  │  │
    │   │ │ │   │  │                      │   │ │ │   │  │
    └───┼─┼─┼───┘  │                      └───┼─┼─┼───┘  │
        │ │ │      │                          │ │ │      │
      G Y SB      │                        G Y SB      │
        │ │ │      │                          │ │ │      │
    ┌───┴─┴─┴──┐   │                      ┌───┴─┴─┴──┐   │
    │  LEFT    │   │                      │  RIGHT   │   │
    │ THRUSTER │   │                      │ THRUSTER │   │
    │  (CCW)   │   │                      │  (CW)    │   │
    └──────────┘   │                      └──────────┘   │
                   │                                     │
              ┌────┴─────────────────────────────────────┘
              │
              │  ┌──────────────────┐
              └──│   LiPo Battery   │
                 │   4S 14.8V       │
                 │   (+)    (-)     │
                 └──┬────────┬──────┘
                    │        │
                    └── B+ ──┘
```

---

## ESC Pinout

```
     MOTOR SIDE                          POWER / SIGNAL SIDE
  ┌──────────────┐                    ┌──────────────────┐
  │              │                    │                  │
  │  ██  black 1 ├── Phase U ── green │  red   ─── LiPo (+) ──┐
  │  ██  black 2 ├── Phase V ── yellow│  black  ─── LiPo (-) ──┤
  │  ██  black 3 ├── Phase W ── skybl │                        │
  │              │                    │  brown  ─── Pi GND ────┤
  │   APISQUEEN  │                    │  red    ─── NOT USED   │
  │   30A ESC    │                    │  orange ─── Pi GPIO    │
  │              │                    │                        │
  └──────────────┘                    └────────────────────────┘
```

---

## Raspberry Pi GPIO Header (40-pin)

```
                    ┌─────┐  ┌─────┐
            3.3V ── │ 1  2 │ ── 5V
            GPIO2 ── │ 3  4 │ ── 5V
            GPIO3 ── │ 5  6 │ ── GND ◄── LEFT ESC brown
            GPIO4 ── │ 7  8 │ ── GPIO14
              GND ── │ 9 10 │ ── GPIO15
           GPIO17 ── │11 12 │ ── GPIO18
           GPIO27 ── │13 14 │ ── GND ◄── RIGHT ESC brown
           GPIO22 ── │15 16 │ ── GPIO23 ── RCWL TRIG
             3.3V ── │17 18 │ ── GPIO24 ── RCWL ECHO
           GPIO10 ── │19 20 │ ── GND
            GPIO9 ── │21 22 │ ── GPIO25
           GPIO11 ── │23 24 │ ── GPIO8
              GND ── │25 26 │ ── GPIO7
             ID_SD ── │27 28 │ ── ID_SC
            GPIO5 ── │29 30 │ ── GND
            GPIO6 ── │31 32 │ ── GPIO12 ◄── LEFT ESC orange
           GPIO13 ◄── │33 34 │ ── GND       RIGHT ESC orange
           GPIO19 ── │35 36 │ ── GPIO16
           GPIO26 ── │37 38 │ ── GPIO20
              GND ── │39 40 │ ── GPIO21
                    └─────┘  └─────┘

    ◄──  = input to Pi from ESC
    ──◇  = output from Pi to RCWL
```

---

## RCWL-1655 Wiring

```
          RCWL-1655
        ┌──────────┐
        │          │
  5V ───┤ VCC      │
 GND ───┤ GND      │
        │          │
GPIO23──┤ TRIG     │
        │          │
        │ ECHO ───┼──┬── GPIO24 (Pi)
        │          │  │
        └──────────┘  │
                 4.7kΩ │
                       ├── GND
                 10kΩ  │
                       │
                       ┘
     (voltage divider: step 5V echo → 3.3V Pi-safe)
```

---

## Differential Steering

```
              FORWARD
                 ▲
                 │
    LEFT CCW ────┼──── RIGHT CCW
    (pulls)      │      (pulls)
      ◄────────  │  ────────►
                 │
          ┌──────┴──────┐
          │    DUCK     │
          │    HULL     │
          └─────────────┘

              REVERSE
                 │
                 ▼
    LEFT CW ─────┼──── RIGHT CW
    (pushes)     │     (pushes)
      ────────►  │  ◄────────
                 │

    LEFT TURN:   LEFT reverse + RIGHT forward
    RIGHT TURN:  LEFT forward + RIGHT reverse
```

---

## U01 Thruster Dimensions

```
        ┌─────────────────────┐
        │                     │
        │     MOUNTING        │
  75mm  │       FACE          │ 75mm
        │    ╔═══════════╗    │
        │    ║  ╭─────╮  ║    │
        │    ║  │  ○  │  ║    │ ◄── propeller
        │    ║  ╰─────╯  ║    │
        │    ╚═══════════╝    │
        │                     │
        └──────────┬──────────┘
                   │
                   │ 330mm cable
                   │
              ┌────┴────┐
              │   ESC   │
              └─────────┘

    4x M3 mounting screws on 75×75mm square pattern
    Total thruster weight: 178g each
     Thrust: 2Kg at 16V (4S)
```

---

## Quick Test with CCPM Servo Tester (No Pi)

```
    ┌──────────────────────────┐
    │     LiPo 3S-4S           │
    │   (+)          (-)       │
    └────┬────────────┬────────┘
         │ RED        │ BLACK
         │            │
    ┌────┴────────────┴────────┐
    │        ESC               │
    │  B+ [●]          [●] B-  │
    │                          │
    │  [███] 3x black ─────────┼─── U01 THRUSTER
    │                          │      (G Y SB)
    │  [Br] [R] [O] 3-pin     │
    └───┬────┬────┬────────────┘
        │    │    │
     BROWN  RED  ORANGE
     (GND) (5V) (PWM)
        │    │    │
    ┌───┴────┴────┴────────────┐
    │  CH1 ( S  +  - )        │
    │  CH2 ( S  +  - )        │◄── test 2nd ESC here
    │  CH3 ( S  +  - )        │
    │                          │
    │    MAN   NEUTRAL   AUTO  │
    │           (KNOB)         │
    │    CCPM SERVO TESTER     │
    └──────────────────────────┘

    MAN:     knob controls speed  (1000-2000µs)
    NEUTRAL: fixed stop          (1500µs)
    AUTO:    auto sweep range    (1000→2000→1000)
```

---

## Servo Tester Modes

```
         MANUAL mode                          AUTO mode
    Knob position → speed               Sweeps full range
    
    2000µs ┤ ████  (full fwd)           2000µs ┤    ╱╲
           │ ████                                │   ╱  ╲
    1750µs ┤ ████  (half fwd)           1750µs ┤  ╱    ╲
           │ ████                                │ ╱      ╲
    1500µs ┤ ████  (stop)               1500µs ┤╱        ╲
           │                               time ───────────
    1250µs ┤ ████  (half rev)
           │ ████
    1000µs ┤ ████  (full rev)
    
        NEUTRAL mode
    Fixed 1500µs output — motor stopped
```
