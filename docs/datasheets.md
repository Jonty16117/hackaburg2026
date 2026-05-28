# Component Datasheets & Technical References

---

## 1. APISQUEEN U01 Brushless Underwater Thruster

- **Official Product Page:** https://www.underwaterthruster.com/en-de/products/u01-12v-16v-200w-2kg-thrust-brushless-underwater-subsea-thruster-propeller-propulsion-with-bi-directional-control-esc-for-rov-boat
- **3D Model (STP):** https://cdn.shopify.com/s/files/1/0621/5493/2452/files/U1-3D.stp?v=1689601104
- **Two-Set Bundle Page:** https://www.underwaterthruster.com/en-de/products/apisqueen-12v-16v-2kg-thrust-u01-tow-set-brushless-underwater-thruster-propeller-with-bi-directional-control-esc-for-rov-boat
- **No dedicated PDF datasheet exists** — specs are on product pages only.

### Specifications

| Parameter | Value |
|-----------|-------|
| Voltage Range | 12V - 16V DC (2S-4S LiPo) |
| Optimal Voltage | 16V (4S LiPo) |
| Max Power | 390W |
| Max Current | 17A |
| Thrust | ~2 kg (forward/reverse) |
| Motor Type | Brushless DC, outer rotor, 2838 |
| Motor Size | 28mm dia x 38mm length |
| KV Rating | 350 KV |
| Propeller | 60mm, plastic (3-4 blade) |
| Dimensions | 75 x 75 mm (L x D) |
| Weight | 178g (thruster only, including cable) |
| Cable Length | 330mm silicone wire |
| Environment | Freshwater only |
| Depth Rating | ~100m |
| Operating Temp | 0C - 40C |
| Material | Corrosion-resistant plastic + metal |
| CW/CCW | Both variants available; use 1x CW + 1x CCW for counter-rotating |

### Notes
- 24V tolerated short-term only; continuous use causes overheating
- Freshwater only — not saltwater rated
- ESC auto-calibrates — no manual throttle calibration needed

---

## 2. APISQUEEN 30A Bi-Directional ESC

- **Official Product Page:** https://www.underwaterthruster.com/en-de/products/apisqueen-bi-directional-30a-esc-2-4s-5v-1a-bec-electronic-speed-controllers
- **Related 45A ESC Manual (PDF):** https://cdn.shopify.com/s/files/1/0621/5493/2452/files/45A.pdf?v=1679410867

### Specifications

| Parameter | Value |
|-----------|-------|
| Battery | 2-4S LiPo (7.4V - 16.8V) |
| Continuous Current | 30A |
| BEC Output | 5V / 1A |
| Wire Gauge | 14AWG |
| Positive/Negative Wire Length | 95mm |
| Motor 3-Phase Wire Length | 300mm |
| Dimensions | 28 x 15 x 6 mm |
| Weight | 36g |
| Direction | Bi-directional (forward 1.5-2ms, reverse 1.5-1ms) |
| PWM Frequency | 50Hz / 100Hz / 200Hz / 500Hz |
| Protocols | PWM, Oneshot, Multishot, Dshot |
| Waterproof Option | Available (metal housing version) |

### Wiring

| Wire Color | Function |
|-----------|----------|
| Red | Battery positive |
| Black | Battery negative |
| Orange/Yellow | PWM signal input |
| Brown | Signal ground |
| Red (servo plug) | BEC 5V/1A output (do NOT connect to Pi) |
| 3x Black (14AWG) | Motor phase wires A, B, C |

### Notes
- BEC 5V output must NOT be connected to Raspberry Pi — use separate USB power
- All throttle signals >= 500Hz are non-standard
- Firmware optimized specifically for underwater thrusters
- Supports rapid throttle response

---

## 3. RCWL-1655 Ultrasonic Distance Sensor

- **Datasheet PDF:** https://makerhero.com/img/files/download/RCWL-1655-Datasheet.pdf
- **Scribd Mirror:** https://www.scribd.com/document/901894038/RCWL-1655-Datasheet

### Specifications

| Parameter | Value |
|-----------|-------|
| Chipset | RCWL-9631 |
| Operating Voltage | 2.8V - 5.5V DC |
| Working Current | < 8mA (3.5mA standby, 30mA peak) |
| Probe Frequency | 40 kHz |
| Detection Range | 20cm - 600cm (at 5V), 20cm - 400cm (at 3.3V) |
| Blind Zone | 20cm |
| Resolution | 1mm |
| Accuracy | ~1cm (long range) |
| Measuring Angle | 75 degrees |
| Working Temperature | -20C to +70C |
| Module Dimensions | L42 x W29 x H12 mm |

### Pinout

| Pin | Function |
|-----|----------|
| VCC | Power: 3.0V - 5.5V |
| Trig / RX | Trigger input (GPIO mode) or UART RX |
| Echo / TX | Echo output (GPIO mode) or UART TX |
| GND | Ground |

### Communication Modes (select via resistor)

1. **GPIO (Default)** — Compatible with HC-SR04 / JSN-SR04T
   - 10us+ trigger pulse on Trig pin
   - Distance = (pulse_us * 34) / 1000 / 2 cm
2. **UART** — Requires soldering 10k resistor across R7 pads
   - Send `0x55`, sensor replies 3 bytes (24-bit) distance, no checksum
3. **I2C**
4. **1-Wire (Single Bus)**

### Raspberry Pi Interface

- **Logic Level:** TTL, 3.3V/5V compatible
- **Voltage Divider Required:** 4.7k + 10k on ECHO pin to step 5V down to 3.3V
- **GPIO Mode Pinout:** BCM 23 (TRIG), BCM 24 (ECHO via divider)
- **Waterproofing:** Transducer head is waterproof; control board is NOT

### Key Differences from JSN-SR04T

| Feature | RCWL-1655 | JSN-SR04T |
|---------|-----------|-----------|
| Distance Data | 24-bit | 16-bit |
| UART Checksum | None | Yes |
| UART Trigger | `0x55` | Multi-byte command |

---

## 4. CCPM 3-Channel Servo Tester

- **Reverse-Engineered Docs (GitHub):** https://github.com/DmitriyDA2020/servo-tester-ATtiny10
- **No official manufacturer datasheet exists** — commodity module, community documented only.

### Specifications

| Parameter | Value |
|-----------|-------|
| Input Voltage | 4.5V - 5.5V |
| PWM Frequency | 50 Hz (20ms period) |
| PWM Range | 580us - 2420us |
| PWM Center (Neutral) | 1500us |
| MCU (Common) | ATtiny10, STM8S003F3, or N76E003 |
| Output Channels | 3 (all driven in parallel) |
| Current Limit | ~2A total (practical limit) |

### Modes of Operation

| Mode | Behavior |
|------|----------|
| **MAN (Analog)** | Servo position follows potentiometer knob position |
| **NEUTRAL (Center)** | All 3 channels output fixed 1.5ms center pulse |
| **AUTO (Test)** | Automatically sweeps min-max with ~150ms pause at endpoints |

Toggle via push button: Analog -> Center -> Auto -> Analog...

### Pinout (ATtiny10 Based)

| Pin | Function |
|-----|----------|
| PB0 | Potentiometer input (ADC) |
| PB1 | PWM output (drives all 3 channels in parallel) |
| PB2 | Push button (pull-up, falling edge interrupt) |

### Output Connectors
- 3x standard 3-pin servo headers (GND / V+ / Signal)
- All signal pins connected together
- Supply voltage passes directly to servo V+ pins (no regulation)

---

## 5. Raspberry Pi GPIO Pinout (Project-Specific)

| BCM Pin | Board Pin | Connected To |
|:-------:|:---------:|-------------|
| 12 | 32 | Left ESC PWM signal (orange) |
| 13 | 33 | Right ESC PWM signal (orange) |
| 23 | 16 | RCWL-1655 TRIG |
| 24 | 18 | RCWL-1655 ECHO (via 4.7k+10k voltage divider) |
| 17, 27, 22 | — | Reserved (status LEDs, etc.) |
| GND (pins 6, 14) | — | Both ESC GND (brown) |

---

## Manufacturer Websites

- **APISQUEEN Official:** https://apisqueen.net
- **APISQUEEN Shop:** https://www.underwaterthruster.com
- **APISQUEEN ROV Shop:** https://www.rovauv.com
- **Support Email:** help@underwaterthruster.com
- **B2B/Wholesale:** lynn@underwaterthruster.com
