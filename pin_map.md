# Pin Map — Diamond C-20 to Teensy 4.0

## RJ45 Connector (viewed from front of laser, pin 1 at left)

```
 ┌─────────────────────────────┐
 │  1   2   3   4   5   6   7   8 │  ← pin numbers
 └─────────────────────────────┘
   MOD +15V LOK TOK VOK INT CE  GND
```

Pin 1 is on the **left** when looking at the front of the laser connector.

---

## Signal Table

| RJ45 Pin | Signal | Direction | Teensy 4.0 Pin | Notes |
|:---:|---|:---:|:---:|---|
| 1 | Modulation | → laser | **4** | Control output; PWM square wave |
| 2 | +15 VDC | ← laser | — | Do not connect |
| 3 | Laser OK | ← laser | **6** | Via voltage divider (see below) |
| 4 | Temperature OK | ← laser | **7** | Via voltage divider |
| 5 | Voltage OK | ← laser | **8** | Via voltage divider |
| 6 | Internal | — | — | Connect to GND or leave open |
| 7 | Control Enable | → laser | **5** | Digital output; HIGH to arm laser |
| 8 | Ground | — | **GND** | Common ground — must be connected |

Use shielded Cat5e/Cat6 cable. Connect the cable shield to the chassis ground of
your controller enclosure. A floating ground is an unsafe condition (manual p.37).

### Power meter analog input (optional)

For closed-loop power regulation, a Thorlabs PM100D with a thermal sensor (e.g.
S314C) can be wired to a free analog input. The PM100D's BNC analog output
swings 0–2 V over the selected range and is safe to feed directly into a 3.3 V
Teensy ADC pin.

| Signal | PM100D | Teensy 4.0 Pin | Notes |
|---|---|:---:|---|
| Meter analog out | BNC center | **14 (A0)** | 12-bit ADC, 0–2 V full scale |
| Meter ground     | BNC shell  | **GND**      | Any GND pad; the one adjacent to pin 14 is convenient |

The Teensy reads this pin with `analogReadAveraging(32)` so the firmware does
its own noise filtering. The PM100D range is **not** read back over the serial
link — set the meter to the range that matches `PM100D_FULL_SCALE_W` in
`laser_controller.py` (default 30 W).

> **Protection note:** The PM100D analog output is rated 0–2 V under normal
> operation but the manual does not guarantee its behaviour under fault
> conditions (overrange, sensor disconnected, power loss). If you want belt-
> and-braces protection against a transient swing above 3.3 V, put a 1 kΩ
> series resistor between the BNC center and pin 14 and a 3.3 V Zener (or BAT54
> clamp) from pin 14 to 3V3. The Teensy ADC's input impedance handles the
> series resistor without measurable error.

---

## Voltage Dividers (REQUIRED on status inputs)

The laser's status outputs are **5V TTL**. The Teensy 4.0 GPIO is **not 5V tolerant**
(absolute maximum input voltage ≈ 3.6 V). A resistor voltage divider must be used
on RJ45 pins 3, 4, and 5 before connecting them to the Teensy.

### Divider values: R1 = 10 kΩ, R2 = 15 kΩ

```
RJ45 pin 3/4/5
      │
     R1 (10 kΩ)
      │
      ├──── Teensy input pin (6 / 7 / 8)
      │
     R2 (15 kΩ)
      │
     GND
```

**Voltage math:**  
V_out = 5 V × 15 / (10 + 15) = **3.0 V**  
Load current at V_HIGH = 5 V / 25 kΩ ≈ 200 µA  
Laser IOH spec = 0.4 mA (200 µA < 400 µA ✓)

This gives a safe TTL HIGH of 3.0 V at the Teensy pin (well above the 2.0 V VIH
threshold, and below the 3.6 V absolute maximum).

### Per-signal divider summary

| RJ45 Pin | Signal | R1 | R2 | Teensy Pin |
|:---:|---|---|---|:---:|
| 3 | Laser OK | 10 kΩ | 15 kΩ | 6 |
| 4 | Temperature OK | 10 kΩ | 15 kΩ | 7 |
| 5 | Voltage OK | 10 kΩ | 15 kΩ | 8 |

---

## Teensy Output Levels

The Teensy 4.0 outputs **3.3 V** logic HIGH. The laser's TTL inputs (Modulation
and Control Enable) have 1 kΩ input impedance and a VIH minimum of 2.0 V.
3.3 V exceeds this — **no level shifting is needed on the output lines**.

---

## Full Wiring Diagram

```
Teensy 4.0                              RJ45 (laser)
──────────                              ────────────
Pin 4  ──────────────────────────────── Pin 1  (Modulation)
Pin 5  ──────────────────────────────── Pin 7  (Control Enable)
                    ┌── R1=10k ──────── Pin 3  (Laser OK)
Pin 6  ─────────────┤
                    └── R2=15k ── GND
                    ┌── R1=10k ──────── Pin 4  (Temperature OK)
Pin 7  ─────────────┤
                    └── R2=15k ── GND
                    ┌── R1=10k ──────── Pin 5  (Voltage OK)
Pin 8  ─────────────┤
                    └── R2=15k ── GND
GND    ──────────────────────────────── Pin 8  (Ground)
                                        Pin 6  (Internal — tie to GND)

Optional:                               PM100D
Pin 14 (A0) ─────────────────────────── BNC center  (Analog out, 0–2 V)
GND    ──────────────────────────────── BNC shell   (Analog out ground)
```

---

## Status LED (Teensy built-in, Pin 13)

| Pattern | Meaning |
|---|---|
| Off | Idle / unexpected state |
| Slow blink (500 ms) | Pre-ionizing (waiting for laser to warm up, ~42 s) |
| Steady on | Ready or Enabled — laser armed but not emitting |
| Double-pulse (1 s period) | Running — laser emitting |
| Fast blink (100 ms) | Fault — laser stopped; check status over serial |
