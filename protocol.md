# Serial Communication Protocol — Diamond C-20 Laser Controller

## Transport

| Parameter | Value |
|---|---|
| Interface | USB CDC (virtual serial port) |
| Baud rate | 115200 (nominal; irrelevant for USB CDC) |
| Line ending | `\n` (LF) — `\r\n` also accepted |
| Encoding | ASCII |
| Case | Commands are case-insensitive |

---

## Message Types

Every line received from the firmware begins with one of four prefixes:

| Prefix | Direction | Meaning |
|---|---|---|
| `OK` | firmware → host | Command succeeded |
| `ERR` | firmware → host | Command failed |
| `STATUS` | firmware → host | Informational / query result line |
| `FAULT` | firmware → host | Unsolicited fault or fault-cleared notification |
| `BOOT` | firmware → host | Emitted once at power-up |

`OK` and `ERR` are always the **last** line of a response. `STATUS` lines may
precede an `OK` (multi-line responses). `FAULT` messages are never preceded by
a command — they arrive asynchronously.

---

## System State Machine

```
         power on
            │
            ▼
    ┌─────────────────┐
    │  PREIONIZING    │  LASER_OK LOW; ~42 s warm-up
    └────────┬────────┘
             │ LASER_OK goes HIGH
             ▼
    ┌─────────────────┐
    │     READY       │  LASER_OK HIGH; CE de-asserted
    └────────┬────────┘
             │ ENABLE
             ▼
    ┌─────────────────┐
    │    ENABLED      │  CE asserted; awaiting ON
    └────────┬────────┘
             │ ON
             ▼
    ┌─────────────────┐
    │    RUNNING      │◄──── POWER (live update)
    └────────┬────────┘
             │ fault detected
             ▼
    ┌─────────────────┐
    │     FAULT       │  CE low; modulation stopped
    └─────────────────┘
             │ FAULT_RESET (pins clear) → READY
             │ ENABLE (from READY)      → ENABLED
```

**OFF** transitions RUNNING → ENABLED (CE stays high).  
**DISABLE** transitions ENABLED/RUNNING → READY.  
Faults in any active state trigger an immediate transition to FAULT.

---

## Commands

### ENABLE

Assert the Control Enable pin. The laser must have completed pre-ionization first.

```
→ ENABLE
← OK ENABLED
```

Errors:
```
← ERR NOT_READY          (still pre-ionizing)
← ERR FAULT_ACTIVE       (send FAULT_RESET first)
```

---

### DISABLE

De-assert Control Enable and stop modulation. Safe to call at any time.

```
→ DISABLE
← OK DISABLED
```

---

### ON [duration_ms]

Start modulation at the currently configured power level. An optional integer
argument specifies a duration in milliseconds after which the laser will shut
off automatically.

```
→ ON
← OK ON

→ ON 5000
← OK ON 5000
   ... (5 seconds later, unsolicited) ...
   STATUS TIMEOUT_OFF
```

Errors:
```
← ERR NOT_ENABLED        (send ENABLE first)
← ERR FAULT_ACTIVE       (send FAULT_RESET first)
← ERR POWER_ZERO         (set POWER > 0 first)
```

Sending `ON` while already RUNNING resets the timed-off counter if a duration
was specified.

---

### OFF

Stop modulation. Control Enable remains asserted (laser stays armed).

```
→ OFF
← OK OFF
```

---

### POWER \<fraction\>

Set the output power as a fraction of maximum (0.0 – 1.0). If the laser is
currently RUNNING the new power level takes effect immediately.

```
→ POWER 0.5
← OK POWER 0.5000

→ POWER 0.01
← OK POWER 0.0100
```

Errors:
```
← ERR INVALID_POWER      (value outside 0.0–1.0)
```

**Power to PWM mapping:**

Let `f` = power fraction, `F_MAX` = 25 000 Hz, `W_MIN` = 1 µs.

| Condition | Frequency | Duty cycle |
|---|---|---|
| f = 0 | — | Output LOW (no pulses) |
| f = 1 | — | Output HIGH (true CW) |
| f ≥ 0.025 | F_MAX (25 kHz) | f |
| 0 < f < 0.025 | f / W_MIN = f × 10⁶ Hz | f |

The threshold 0.025 = W_MIN × F_MAX ensures the pulse width never falls below
1 µs regardless of power setting.

Examples:

| POWER | Frequency | Duty | Pulse width |
|---|---|---|---|
| 1.000 | CW | 100 % | ∞ |
| 0.500 | 25 000 Hz | 50.0 % | 20 µs |
| 0.100 | 25 000 Hz | 10.0 % | 4 µs |
| 0.025 | 25 000 Hz | 2.5 % | 1 µs |
| 0.010 | 10 000 Hz | 1.0 % | 1 µs |
| 0.001 |  1 000 Hz | 0.1 % | 1 µs |

---

### STATUS

Query the full system status. Returns several `STATUS` lines followed by `OK STATUS`.

```
→ STATUS
← STATUS STATE RUNNING
← STATUS POWER 0.5000
← STATUS LASER_OK HIGH
← STATUS TEMP_OK HIGH
← STATUS VOLTAGE_OK HIGH
← STATUS FAULTS NONE
← STATUS FREQ 25000
← STATUS DUTY 0.5000
← OK STATUS
```

`STATUS FREQ` and `STATUS DUTY` show `--` when the laser is not running, and
`CW` / `1.0000` when running at full power (DC output).

---

### FAULT_RESET

Attempt to recover from a fault state. The CE line was lowered when the fault
occurred; this command verifies pins are clear and transitions back to READY.

```
→ FAULT_RESET
← OK FAULT_RESET -- send ENABLE to re-arm
```

Errors:
```
← ERR NOT_IN_FAULT
← ERR TEMP_TOO_HIGH      (wait for laser to cool)
← ERR FAULTS_PERSIST 0x01   (hex bitmask of remaining faults)
```

Fault bitmask:
| Bit | Value | Fault |
|---|---|---|
| 0 | 0x01 | LASER_OK |
| 1 | 0x02 | TEMP |
| 2 | 0x04 | VOLTAGE |

After a successful reset, send `ENABLE` to re-arm and `ON` to resume.

---

### IDENT

Report firmware version and build date.

```
→ IDENT
← OK IDENT laser_controller v1.0 built Apr 17 2026 12:34:56
```

---

## Unsolicited Messages

These lines arrive without a preceding command. A host application should read
all incoming lines, not just responses to commands, to catch these.

| Message | Meaning |
|---|---|
| `BOOT OK` | Firmware started |
| `STATUS PREIONIZING` | Pre-ionization phase started |
| `STATUS READY` | Pre-ionization complete; ready to ENABLE |
| `STATUS TIMEOUT_OFF` | Timed ON expired; laser stopped |
| `FAULT LASER_OK` | Composite fault (VSWR / temperature latch / discharge) |
| `FAULT TEMP` | Temperature warning tripped (>60 °C) |
| `FAULT VOLTAGE` | Supply voltage out of range (<43 V or >55 V) |
| `FAULT LASER_OK TEMP` | Multiple simultaneous faults |
| `FAULT CLEARED` | All fault pins returned HIGH; send ENABLE to re-arm |
| `FAULT PREIONIZE_TIMEOUT` | LASER_OK did not go HIGH within 60 s |

---

## Startup Sequence

The recommended sequence after the laser is powered on:

```python
# 1. Open serial port.
# 2. Wait for "STATUS READY" (LASER_OK goes HIGH after ~42 s warm-up).
# 3. Set desired power.
send("POWER 0.5")   # → OK POWER 0.5000

# 4. Assert Control Enable.
send("ENABLE")      # → OK ENABLED

# 5. Start emission.
send("ON")          # → OK ON
# Firmware now waits 1 s before trusting LASER_OK to avoid false VSWR reports.

# 6. Stop.
send("OFF")         # → OK OFF
send("DISABLE")     # → OK DISABLED (optional; CE is lowered)
```

---

## Fault Recovery Sequence

When a `FAULT` message is received:

```python
# 1. Emission has already been stopped by firmware.
# 2. Identify fault from FAULT message.
# 3. If TEMP fault: wait until temperature drops (watch for FAULT CLEARED).
# 4. Attempt reset.
send("FAULT_RESET")  # → OK FAULT_RESET  or  ERR FAULTS_PERSIST ...
# 5. Re-arm.
send("ENABLE")       # → OK ENABLED
# 6. Resume.
send("ON")           # → OK ON
```

---

## Notes for Implementors

- Strip `\r` and leading/trailing whitespace before parsing responses.
- The firmware strips `\r` on input, so `\r\n` line endings are safe to send.
- Unknown commands return `ERR UNKNOWN_COMMAND <verb>`.
- `POWER` may be sent at any time including before `ENABLE`; it is stored and
  applied when `ON` is issued (or immediately if already RUNNING).
- Do not use Control Enable for modulation — its response time is milliseconds,
  not microseconds. Always modulate via the `ON`/`OFF` commands (Pin 1).
