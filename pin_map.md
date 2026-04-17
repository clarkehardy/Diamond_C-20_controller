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
