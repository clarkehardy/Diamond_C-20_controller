// =============================================================================
// Diamond C-20 CO2 Laser Controller — Teensy 4.0 Firmware
// =============================================================================
// Accepts ASCII serial commands over USB CDC and outputs TTL control signals
// to the laser's RJ45 interface. Monitors three status pins and stops emission
// immediately on any hardware fault.
//
// Compile with: Teensyduino add-on for Arduino IDE
//   Board: Teensy 4.0
//   USB Type: Serial
//   CPU Speed: 600 MHz (or any)
//
// See pin_map.md for wiring and protocol.md for the serial command reference.
// =============================================================================

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
#define PIN_MODULATION    4    // PWM output  → RJ45 Pin 1 (Modulation)
#define PIN_CTRL_ENABLE   5    // Digital out → RJ45 Pin 7 (Control Enable)
#define PIN_LASER_OK      6    // Digital in  ← RJ45 Pin 3 (Laser OK)   via divider
#define PIN_TEMP_OK       7    // Digital in  ← RJ45 Pin 4 (Temp OK)    via divider
#define PIN_VOLTAGE_OK    8    // Digital in  ← RJ45 Pin 5 (Voltage OK) via divider
#define PIN_STATUS_LED   13    // Built-in LED

// ---------------------------------------------------------------------------
// PWM / modulation parameters  (edit these to change operating limits)
// ---------------------------------------------------------------------------
#define MAX_FREQ_HZ       100000   // Maximum modulation frequency (Hz)  [manual p.44]
#define MIN_PULSE_US      2        // Minimum pulse width (microseconds) [manual p.44]
#define PWM_RESOLUTION    12       // PWM bit depth; 4096 counts

// Derived: below this power fraction the pulse-width-hold regime activates.
// POWER_THRESHOLD = MIN_PULSE_US * 1e-6 * MAX_FREQ_HZ = 0.025 (2.5%)
#define POWER_THRESHOLD   ((float)MIN_PULSE_US * 1e-6f * (float)MAX_FREQ_HZ)
#define PWM_MAX_COUNT     ((1 << PWM_RESOLUTION) - 1)   // 4095

// ---------------------------------------------------------------------------
// Timing constants (milliseconds)
// ---------------------------------------------------------------------------
#define PREIONIZE_TIMEOUT_MS    60000UL  // Safety ceiling; laser takes ~42s [p.43]
#define POST_ENABLE_SETTLE_MS    1000UL  // Ignore LASER_OK after CE+MOD asserted [p.49]
#define STATUS_POLL_INTERVAL_MS    50UL  // Status pin poll period

// ---------------------------------------------------------------------------
// Fault bitmask
// ---------------------------------------------------------------------------
#define FAULT_NONE      0x00
#define FAULT_LASER_OK  0x01   // LASER_OK pin went LOW
#define FAULT_TEMP      0x02   // TEMP_OK pin went LOW
#define FAULT_VOLTAGE   0x04   // VOLTAGE_OK pin went LOW

// ---------------------------------------------------------------------------
// Serial buffer
// ---------------------------------------------------------------------------
#define SERIAL_BUF_SIZE  64

// ---------------------------------------------------------------------------
// System states
// ---------------------------------------------------------------------------
enum SystemState {
    STATE_PREIONIZING,  // Waiting for LASER_OK to go HIGH after power-up (~42s)
    STATE_READY,        // LASER_OK high; CE not asserted
    STATE_ENABLED,      // CE asserted; awaiting ON command
    STATE_RUNNING,      // CE asserted + modulation active
    STATE_FAULT         // Fault detected; CE low, modulation stopped
};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
SystemState systemState       = STATE_PREIONIZING;
unsigned long stateEnteredAt  = 0;

float    targetPower          = 0.0f;
uint32_t currentFreq          = MAX_FREQ_HZ;
float    currentDuty          = 0.0f;
bool     cwMode               = false;   // True when f==1.0 (DC HIGH)

bool     timedOffActive       = false;
unsigned long timedOffEnd_ms  = 0;

uint8_t  currentFaults        = FAULT_NONE;
bool     inPostEnableSettle   = false;
unsigned long settleEnd_ms    = 0;

unsigned long lastStatusPoll  = 0;

char     serialBuf[SERIAL_BUF_SIZE];
uint8_t  serialBufPos         = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void applyPWM(float f);
void stopModulation();
void handleFault(uint8_t faults);
void handleCommand(char *cmd);
void cmdEnable();
void cmdDisable();
void cmdOn(long duration_ms);
void cmdOff();
void cmdPower(float f);
void cmdStatus();
void cmdFaultReset();
void cmdIdent();
void updateStatusLED();
void pollStatusPins();
String faultNames(uint8_t faults);

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
    // Wait up to 2 s for USB CDC to enumerate; avoids dropped boot messages.
    Serial.begin(115200);
    unsigned long t = millis();
    while (!Serial && (millis() - t) < 2000) {}

    // Configure PWM: 12-bit resolution, max frequency on the modulation pin.
    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(PIN_MODULATION, MAX_FREQ_HZ);

    // Outputs start LOW / de-asserted.
    pinMode(PIN_MODULATION,  OUTPUT); digitalWrite(PIN_MODULATION,  LOW);
    pinMode(PIN_CTRL_ENABLE, OUTPUT); digitalWrite(PIN_CTRL_ENABLE, LOW);
    pinMode(PIN_STATUS_LED,  OUTPUT); digitalWrite(PIN_STATUS_LED,  LOW);

    // Inputs — no internal pull-ups; the voltage dividers provide defined levels.
    pinMode(PIN_LASER_OK,   INPUT);
    pinMode(PIN_TEMP_OK,    INPUT);
    pinMode(PIN_VOLTAGE_OK, INPUT);

    systemState    = STATE_PREIONIZING;
    stateEnteredAt = millis();

    Serial.println("BOOT OK");
    Serial.println("STATUS PREIONIZING");
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
    // ------------------------------------------------------------------
    // 1. Serial input: accumulate into buffer; dispatch on newline.
    // ------------------------------------------------------------------
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBufPos > 0) {
                serialBuf[serialBufPos] = '\0';
                handleCommand(serialBuf);
                serialBufPos = 0;
            }
        } else {
            if (serialBufPos < SERIAL_BUF_SIZE - 1) {
                serialBuf[serialBufPos++] = c;
            } else {
                // Buffer overflow — discard and report.
                serialBufPos = 0;
                Serial.println("ERR BUFFER_OVERFLOW");
            }
        }
    }

    // ------------------------------------------------------------------
    // 2. Status pin polling (every STATUS_POLL_INTERVAL_MS).
    // ------------------------------------------------------------------
    if (millis() - lastStatusPoll >= STATUS_POLL_INTERVAL_MS) {
        pollStatusPins();
        lastStatusPoll = millis();
    }

    // ------------------------------------------------------------------
    // 3. State-specific logic.
    // ------------------------------------------------------------------
    switch (systemState) {

        case STATE_PREIONIZING:
            // Wait for LASER_OK to go HIGH.
            if (digitalRead(PIN_LASER_OK) == HIGH) {
                systemState    = STATE_READY;
                stateEnteredAt = millis();
                Serial.println("STATUS READY");
            } else if (millis() - stateEnteredAt > PREIONIZE_TIMEOUT_MS) {
                Serial.println("FAULT PREIONIZE_TIMEOUT");
                // Stay in PREIONIZING; operator must investigate hardware.
            }
            break;

        case STATE_ENABLED:
        case STATE_RUNNING:
            // Check post-enable settle window expiry (suppress false LASER_OK faults).
            if (inPostEnableSettle && millis() >= settleEnd_ms) {
                inPostEnableSettle = false;
            }
            // Check timed-off expiry.
            if (timedOffActive && millis() >= timedOffEnd_ms) {
                timedOffActive = false;
                stopModulation();
                Serial.println("STATUS TIMEOUT_OFF");
            }
            break;

        default:
            break;
    }

    // ------------------------------------------------------------------
    // 4. Status LED.
    // ------------------------------------------------------------------
    updateStatusLED();
}

// ===========================================================================
// pollStatusPins()
// ===========================================================================
void pollStatusPins() {
    // During post-enable settle, only report non-LASER_OK faults.
    // (LASER_OK can take up to 1 second to stabilise after CE+MOD — manual p.49)
    uint8_t newFaults = FAULT_NONE;
    if (digitalRead(PIN_LASER_OK)   == LOW) newFaults |= FAULT_LASER_OK;
    if (digitalRead(PIN_TEMP_OK)    == LOW) newFaults |= FAULT_TEMP;
    if (digitalRead(PIN_VOLTAGE_OK) == LOW) newFaults |= FAULT_VOLTAGE;

    // Mask LASER_OK during settle window.
    uint8_t effectiveFaults = newFaults;
    if (inPostEnableSettle) {
        effectiveFaults &= ~FAULT_LASER_OK;
    }

    if (effectiveFaults != currentFaults) {
        uint8_t prev   = currentFaults;
        currentFaults  = effectiveFaults;

        if (effectiveFaults != FAULT_NONE) {
            // New fault(s) appeared.
            if (systemState == STATE_RUNNING || systemState == STATE_ENABLED) {
                handleFault(effectiveFaults);
            } else if (systemState != STATE_FAULT && systemState != STATE_PREIONIZING) {
                // Not running, but flag it anyway.
                Serial.print("FAULT ");
                Serial.println(faultNames(effectiveFaults));
            }
        } else if (prev != FAULT_NONE && systemState == STATE_FAULT) {
            // All faults cleared.
            Serial.println("FAULT CLEARED -- send ENABLE to re-arm");
        }
    }
}

// ===========================================================================
// applyPWM(float f)
//   f = 0.0  → pin stays LOW  (no emission)
//   f = 1.0  → pin stays HIGH (true CW)
//   0 < f < 1 → PWM in one of two regimes:
//     f >= POWER_THRESHOLD: MAX_FREQ, duty = f
//     f <  POWER_THRESHOLD: duty = f, freq = f / (MIN_PULSE_US * 1e-6)
// ===========================================================================
void applyPWM(float f) {
    cwMode = false;

    if (f <= 0.0f) {
        // Restore timer control of the pin, then force duty to 0.
        analogWriteFrequency(PIN_MODULATION, MAX_FREQ_HZ);
        analogWrite(PIN_MODULATION, 0);
        currentFreq = MAX_FREQ_HZ;
        currentDuty = 0.0f;
        return;
    }

    if (f >= 1.0f) {
        // True CW: release PWM timer, drive pin HIGH with GPIO.
        // This ensures a clean DC level with no PWM glitches.
        analogWrite(PIN_MODULATION, 0);
        digitalWrite(PIN_MODULATION, HIGH);
        cwMode      = true;
        currentFreq = 0;        // Not applicable in CW mode
        currentDuty = 1.0f;
        return;
    }

    // --- Intermediate power ---
    uint32_t freq;
    float    duty;

    if (f >= POWER_THRESHOLD) {
        // High-power regime: hold frequency at MAX, vary duty cycle.
        freq = MAX_FREQ_HZ;
        duty = f;
    } else {
        // Low-power regime: hold pulse width at MIN_PULSE_US, reduce frequency.
        // freq = f / (MIN_PULSE_US [s]) so that duty = freq * MIN_PULSE_US[s] = f
        freq = (uint32_t)(f / ((float)MIN_PULSE_US * 1e-6f) + 0.5f);
        if (freq < 1)           freq = 1;
        if (freq > MAX_FREQ_HZ) freq = MAX_FREQ_HZ;
        duty = (float)freq * (float)MIN_PULSE_US * 1e-6f;
    }

    // Compute PWM counts (must call analogWriteFrequency BEFORE analogWrite).
    uint32_t counts = (uint32_t)(duty * (float)(PWM_MAX_COUNT + 1) + 0.5f);
    if (counts < 1)             counts = 1;
    if (counts > PWM_MAX_COUNT) counts = PWM_MAX_COUNT;

    analogWriteFrequency(PIN_MODULATION, freq);
    analogWrite(PIN_MODULATION, counts);

    currentFreq = freq;
    currentDuty = duty;
}

// ===========================================================================
// stopModulation()
// ===========================================================================
void stopModulation() {
    applyPWM(0.0f);
    timedOffActive = false;
    if (systemState == STATE_RUNNING) {
        systemState = STATE_ENABLED;
    }
}

// ===========================================================================
// handleFault()
// ===========================================================================
void handleFault(uint8_t faults) {
    // Stop emission first.
    applyPWM(0.0f);
    // Lower CE — required to clear temperature latch (manual p.48).
    digitalWrite(PIN_CTRL_ENABLE, LOW);
    timedOffActive = false;
    systemState    = STATE_FAULT;

    Serial.print("FAULT ");
    Serial.println(faultNames(faults));
}

// ===========================================================================
// Command dispatcher
// ===========================================================================
void handleCommand(char *raw) {
    // Trim leading whitespace.
    while (*raw == ' ' || *raw == '\t') raw++;

    // Convert to uppercase in-place.
    for (char *p = raw; *p; p++) {
        if (*p >= 'a' && *p <= 'z') *p -= 32;
    }

    // Split on first space to get verb and optional argument string.
    char *verb = raw;
    char *args = raw;
    while (*args && *args != ' ') args++;
    if (*args == ' ') {
        *args = '\0';
        args++;
        while (*args == ' ') args++;  // trim leading spaces in args
    } else {
        args = NULL;
    }

    if      (strcmp(verb, "ENABLE")      == 0) cmdEnable();
    else if (strcmp(verb, "DISABLE")     == 0) cmdDisable();
    else if (strcmp(verb, "ON")          == 0) {
        long duration = (args && *args) ? atol(args) : 0;
        cmdOn(duration);
    }
    else if (strcmp(verb, "OFF")         == 0) cmdOff();
    else if (strcmp(verb, "POWER")       == 0) {
        float f = (args && *args) ? atof(args) : -1.0f;
        cmdPower(f);
    }
    else if (strcmp(verb, "STATUS")      == 0) cmdStatus();
    else if (strcmp(verb, "FAULT_RESET") == 0) cmdFaultReset();
    else if (strcmp(verb, "IDENT")       == 0) cmdIdent();
    else {
        Serial.print("ERR UNKNOWN_COMMAND ");
        Serial.println(verb);
    }
}

// ===========================================================================
// Command implementations
// ===========================================================================

void cmdEnable() {
    if (systemState == STATE_PREIONIZING) {
        Serial.println("ERR NOT_READY");
        return;
    }
    if (systemState == STATE_FAULT) {
        Serial.println("ERR FAULT_ACTIVE -- send FAULT_RESET first");
        return;
    }
    if (systemState == STATE_ENABLED || systemState == STATE_RUNNING) {
        // Already enabled; idempotent.
        Serial.println("OK ENABLED");
        return;
    }

    digitalWrite(PIN_CTRL_ENABLE, HIGH);
    systemState        = STATE_ENABLED;
    stateEnteredAt     = millis();
    inPostEnableSettle = false;   // Settle only starts when modulation is also asserted.
    Serial.println("OK ENABLED");
}

void cmdDisable() {
    stopModulation();
    digitalWrite(PIN_CTRL_ENABLE, LOW);
    timedOffActive     = false;
    inPostEnableSettle = false;
    if (systemState != STATE_PREIONIZING && systemState != STATE_FAULT) {
        systemState = STATE_READY;
    }
    Serial.println("OK DISABLED");
}

void cmdOn(long duration_ms) {
    if (systemState != STATE_ENABLED && systemState != STATE_RUNNING) {
        if (systemState == STATE_FAULT) {
            Serial.println("ERR FAULT_ACTIVE -- send FAULT_RESET first");
        } else {
            Serial.println("ERR NOT_ENABLED -- send ENABLE first");
        }
        return;
    }
    if (targetPower <= 0.0f) {
        Serial.println("ERR POWER_ZERO -- set POWER before ON");
        return;
    }

    applyPWM(targetPower);
    systemState = STATE_RUNNING;

    // Start or reset post-enable settle window.
    inPostEnableSettle = true;
    settleEnd_ms       = millis() + POST_ENABLE_SETTLE_MS;

    if (duration_ms > 0) {
        timedOffActive = true;
        timedOffEnd_ms = millis() + (unsigned long)duration_ms;
        Serial.print("OK ON ");
        Serial.println(duration_ms);
    } else {
        timedOffActive = false;
        Serial.println("OK ON");
    }
}

void cmdOff() {
    stopModulation();
    inPostEnableSettle = false;
    Serial.println("OK OFF");
}

void cmdPower(float f) {
    if (f < 0.0f || f > 1.0f) {
        Serial.println("ERR INVALID_POWER -- must be 0.0 to 1.0");
        return;
    }
    targetPower = f;

    // Apply immediately if laser is running.
    if (systemState == STATE_RUNNING) {
        applyPWM(targetPower);
    }

    Serial.print("OK POWER ");
    Serial.println(targetPower, 4);
}

void cmdStatus() {
    const char *stateName;
    switch (systemState) {
        case STATE_PREIONIZING: stateName = "PREIONIZING"; break;
        case STATE_READY:       stateName = "READY";       break;
        case STATE_ENABLED:     stateName = "ENABLED";     break;
        case STATE_RUNNING:     stateName = "RUNNING";     break;
        case STATE_FAULT:       stateName = "FAULT";       break;
        default:                stateName = "UNKNOWN";     break;
    }

    Serial.print("STATUS STATE ");     Serial.println(stateName);
    Serial.print("STATUS POWER ");     Serial.println(targetPower, 4);
    Serial.print("STATUS LASER_OK ");  Serial.println(digitalRead(PIN_LASER_OK)   ? "HIGH" : "LOW");
    Serial.print("STATUS TEMP_OK ");   Serial.println(digitalRead(PIN_TEMP_OK)    ? "HIGH" : "LOW");
    Serial.print("STATUS VOLTAGE_OK "); Serial.println(digitalRead(PIN_VOLTAGE_OK) ? "HIGH" : "LOW");
    Serial.print("STATUS FAULTS ");
    if (currentFaults == FAULT_NONE) {
        Serial.println("NONE");
    } else {
        Serial.println(faultNames(currentFaults));
    }
    if (systemState == STATE_RUNNING && !cwMode) {
        Serial.print("STATUS FREQ ");  Serial.println(currentFreq);
        Serial.print("STATUS DUTY ");  Serial.println(currentDuty, 4);
    } else if (systemState == STATE_RUNNING && cwMode) {
        Serial.println("STATUS FREQ CW");
        Serial.println("STATUS DUTY 1.0000");
    } else {
        Serial.println("STATUS FREQ --");
        Serial.println("STATUS DUTY --");
    }
    Serial.println("OK STATUS");
}

void cmdFaultReset() {
    if (systemState != STATE_FAULT) {
        Serial.println("ERR NOT_IN_FAULT");
        return;
    }

    // Read raw pin states before attempting reset.
    uint8_t rawFaults = FAULT_NONE;
    if (digitalRead(PIN_LASER_OK)   == LOW) rawFaults |= FAULT_LASER_OK;
    if (digitalRead(PIN_TEMP_OK)    == LOW) rawFaults |= FAULT_TEMP;
    if (digitalRead(PIN_VOLTAGE_OK) == LOW) rawFaults |= FAULT_VOLTAGE;

    if (rawFaults & FAULT_TEMP) {
        Serial.println("ERR TEMP_TOO_HIGH -- wait for laser to cool before reset");
        return;
    }

    // Cycle CE to clear temperature latch (manual p.48).
    // CE is already LOW (set by handleFault). Briefly pulse it LOW→HIGH→LOW is
    // not required; it's already low. We will raise it again via ENABLE after
    // confirming faults are gone.
    delay(200);  // Brief pause before re-reading pins.

    rawFaults = FAULT_NONE;
    if (digitalRead(PIN_LASER_OK)   == LOW) rawFaults |= FAULT_LASER_OK;
    if (digitalRead(PIN_TEMP_OK)    == LOW) rawFaults |= FAULT_TEMP;
    if (digitalRead(PIN_VOLTAGE_OK) == LOW) rawFaults |= FAULT_VOLTAGE;

    if (rawFaults != FAULT_NONE) {
        Serial.print("ERR FAULTS_PERSIST ");
        char hex[8];
        snprintf(hex, sizeof(hex), "0x%02X", rawFaults);
        Serial.println(hex);
        return;
    }

    currentFaults = FAULT_NONE;
    systemState   = STATE_READY;
    Serial.println("OK FAULT_RESET -- send ENABLE to re-arm");
}

void cmdIdent() {
    Serial.print("OK IDENT laser_controller v1.0 built ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);
}

// ===========================================================================
// Helpers
// ===========================================================================

String faultNames(uint8_t faults) {
    String s = "";
    if (faults & FAULT_LASER_OK) s += "LASER_OK ";
    if (faults & FAULT_TEMP)     s += "TEMP ";
    if (faults & FAULT_VOLTAGE)  s += "VOLTAGE ";
    s.trim();
    return s;
}

void updateStatusLED() {
    // Simple non-blocking blink patterns using millis().
    unsigned long t = millis();

    switch (systemState) {
        case STATE_FAULT:
            // Fast blink: 100ms on / 100ms off
            digitalWrite(PIN_STATUS_LED, (t % 200) < 100);
            break;
        case STATE_PREIONIZING:
            // Slow blink: 500ms on / 500ms off
            digitalWrite(PIN_STATUS_LED, (t % 1000) < 500);
            break;
        case STATE_READY:
        case STATE_ENABLED:
            // Steady on
            digitalWrite(PIN_STATUS_LED, HIGH);
            break;
        case STATE_RUNNING:
            // Double-pulse: 100ms on, 100ms off, 100ms on, 700ms off
            {
                unsigned long phase = t % 1000;
                bool on = (phase < 100) || (phase >= 200 && phase < 300);
                digitalWrite(PIN_STATUS_LED, on);
            }
            break;
        default:
            digitalWrite(PIN_STATUS_LED, LOW);
            break;
    }
}
