# Diamond C-20 Laser Controller

Teensy 4.0 firmware and Python driver for the [Coherent Diamond C-20 CO2 laser](https://www.coherent.com). Accepts serial commands over USB, outputs TTL control signals via the laser's RJ45 interface, and monitors hardware fault pins.

## Repository Contents

| File | Description |
|---|---|
| `firmware/laser_controller/laser_controller.ino` | Teensy 4.0 firmware (Teensyduino / Arduino IDE) |
| `laser_controller.py` | Python driver and demo script |
| `pin_map.md` | Wiring guide: Teensy pins → RJ45 connector |
| `protocol.md` | Serial command protocol reference |

## Hardware Requirements

- Teensy 4.0 microcontroller
- Shielded Cat5e/Cat6 RJ45 cable
- Six resistors for voltage dividers on status inputs: **3× 10 kΩ and 3× 15 kΩ**

> **Important:** The laser's status output pins are 5V TTL. The Teensy 4.0 is not 5V tolerant. Voltage dividers are required — see `pin_map.md`.

## Wiring

See [`pin_map.md`](pin_map.md) for the full wiring diagram. Summary:

| Teensy Pin | RJ45 Pin | Signal |
|:---:|:---:|---|
| 4 | 1 | Modulation output |
| 5 | 7 | Control Enable output |
| 6 | 3 | Laser OK input (via 10 kΩ/15 kΩ divider) |
| 7 | 4 | Temperature OK input (via 10 kΩ/15 kΩ divider) |
| 8 | 5 | Voltage OK input (via 10 kΩ/15 kΩ divider) |
| GND | 8 | Ground |

## Loading the Firmware

1. Install [Arduino IDE](https://www.arduino.cc/en/software) and the [Teensyduino](https://www.pjrc.com/teensy/td_download.html) add-on.
2. Open `firmware/laser_controller/laser_controller.ino`.
3. Select **Tools → Board → Teensy 4.0** and **Tools → USB Type → Serial**.
4. Click Upload.

## Python Driver

Install the only dependency:

```bash
pip install pyserial
```

Run the demo:

```bash
# Single 2-second burst at 50% power
python laser_controller.py --port /dev/ttyACM0 --power 0.5 --duration 2000

# Sweep power from 10% to 100% (useful for oscilloscope verification)
python laser_controller.py --port /dev/ttyACM0 --sweep

# Windows
python laser_controller.py --port COM3 --power 0.1 --duration 1000
```

## Serial Protocol

See [`protocol.md`](protocol.md) for the full reference. Quick summary:

| Command | Description |
|---|---|
| `ENABLE` | Assert Control Enable pin |
| `DISABLE` | De-assert Control Enable, stop modulation |
| `POWER <0.0–1.0>` | Set output power as a fraction of maximum |
| `ON [ms]` | Start emission; optional auto-off duration |
| `OFF` | Stop modulation (Control Enable stays asserted) |
| `STATUS` | Query full system status |
| `FAULT_RESET` | Attempt fault recovery |

## Power Control

Power is set as a fraction (0.0–1.0) and translated to a PWM waveform within these hardware limits:

- Maximum frequency: **25 kHz**
- Minimum pulse width: **1 µs**

| Power | Frequency | Duty cycle | Pulse width |
|---|---|---|---|
| 100% | CW (DC HIGH) | — | — |
| 50% | 25 kHz | 50% | 20 µs |
| 10% | 25 kHz | 10% | 4 µs |
| 2.5% | 25 kHz | 2.5% | 1 µs |
| 1% | 10 kHz | 1% | 1 µs |
| 0.1% | 1 kHz | 0.1% | 1 µs |

Below 2.5% the frequency is reduced to keep the pulse width at exactly 1 µs.

## Startup Sequence

After powering the laser, it pre-ionizes for approximately 42 seconds before it can fire. The firmware monitors the Laser OK pin and reports `STATUS READY` when pre-ionization is complete. The Python driver's `wait_for_ready()` method handles this automatically.

## Fault Handling

The firmware polls three status pins every 50 ms. On any fault it immediately stops modulation, lowers Control Enable, and sends an unsolicited `FAULT` message over serial. After the condition clears, send `FAULT_RESET` then `ENABLE` to re-arm.
