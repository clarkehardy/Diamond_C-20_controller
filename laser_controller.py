#!/usr/bin/env python3
"""
Diamond C-20 CO2 Laser Controller — Python driver and demo script.

Requires: pyserial  (pip install pyserial)

Usage:
    python laser_controller.py --port /dev/ttyACM0 --power 0.5 --duration 2000
    python laser_controller.py --port COM3 --power 0.5 --sweep
    python laser_controller.py --port /dev/ttyACM0 --power 0.1 --duration 500

See protocol.md for the full serial command reference.
"""

import argparse
import queue
import sys
import threading
import time
import serial


# ---------------------------------------------------------------------------
# Hard-coded power-meter configuration  (edit these for your setup)
# ---------------------------------------------------------------------------
# PM100D analog output: 0–2 V represents 0 to the currently selected range.
# Set the PM100D range to match PM100D_FULL_SCALE_W below — the firmware has
# no way to read it back.  S314C max is 30 W; default leaves headroom for
# ~18 W reflected off an OD 1.0 ND dump driven by a 20 W laser.
PM100D_FULL_SCALE_W    = 30.0    # W, must match PM100D range setting
PM100D_ANALOG_OUT_FS_V = 2.0     # V at full scale (PM100D spec)

# Ratio of delivered (downstream of the ND filter) to measured (on the meter
# behind the dump).  OD 1.0 reflective ND transmits ~10 % so ratio ≈ 0.1.
# Calibrate against a second meter once and update this number.
TRANSMISSION_RATIO     = 0.1

# Default PID gains.  Tune in-place.  The system gain (W of delivered power per
# unit POWER fraction) depends on the laser, the optics, and the duty regime,
# so these defaults are deliberately conservative.  MAX_STEP caps how much the
# POWER fraction can move in a single update — a hard guard against the loop
# slamming the laser if a parameter is wrong.
FEEDBACK_KP            = 0.05    # 1 W error → +5 % POWER per update
FEEDBACK_KI            = 0.10    # 1 W error sustained → +10 % POWER per second
FEEDBACK_KD            = 0.0
FEEDBACK_MAX_STEP      = 0.01    # max |ΔPOWER| per loop iteration
FEEDBACK_UPDATE_S      = 0.25    # loop period (4 Hz; PM100D analog out ≈ 10 Hz)
FEEDBACK_WARMUP_S      = 3.0     # delay after ON before PID engages; the laser
                                 # takes a couple of seconds to start producing
                                 # light, during which the meter reads ~0 W.


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class LaserError(Exception):
    """Base class for laser controller errors."""


class LaserFault(LaserError):
    """Raised when the firmware reports a hardware fault."""


class NotReady(LaserError):
    """Raised when a command requires a state the laser is not yet in."""


# ---------------------------------------------------------------------------
# LaserPort — thin serial wrapper
# ---------------------------------------------------------------------------

class LaserPort:
    """
    Low-level serial port wrapper. Sends commands and collects responses.

    A single internal reader thread owns all serial reads and routes lines:
      - FAULT / STATUS / BOOT lines with no command in flight → unsolicited callback
      - Everything else (OK, ERR, and STATUS lines while a command is in flight)
        → response queue read by send_command()

    This avoids the race condition that occurs when a background monitor thread
    and send_command() both call readline() and compete for the same bytes.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 5.0):
        self._port_name        = port
        self._baudrate         = baudrate
        self._timeout          = timeout
        self._serial: serial.Serial | None = None
        self._write_lock       = threading.Lock()
        self._response_queue: queue.Queue[str] = queue.Queue()
        self._cmd_in_flight    = threading.Event()
        self._unsolicited_cb   = None
        self._stop_evt         = threading.Event()
        self._reader_thread: threading.Thread | None = None

    def open(self) -> None:
        self._serial = serial.Serial(self._port_name, self._baudrate, timeout=0.1)
        time.sleep(2.0)            # wait for Teensy USB CDC to enumerate
        self._stop_evt.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def set_unsolicited_callback(self, cb) -> None:
        """Register a callback for FAULT/STATUS/BOOT lines not part of a response."""
        self._unsolicited_cb = cb

    def _reader_loop(self) -> None:
        """Single reader thread — the only code that ever calls serial.readline()."""
        while not self._stop_evt.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue

            if self._cmd_in_flight.is_set():
                # A command is pending: all lines go to the response queue so
                # send_command() can collect them (including STATUS response lines).
                self._response_queue.put(line)
            else:
                # No command pending: FAULT/STATUS/BOOT lines are unsolicited.
                if line.startswith(("FAULT", "STATUS", "BOOT")):
                    if self._unsolicited_cb:
                        try:
                            self._unsolicited_cb(line)
                        except Exception as exc:
                            print(f"[reader] callback error: {exc}", file=sys.stderr)
                # Unexpected OK/ERR with no command in flight — discard.

    def send_command(self, cmd: str, timeout: float | None = None) -> list[str]:
        """
        Send a command and collect the full response (up to OK/ERR terminator).
        Returns a list of response lines.
        Raises LaserError if the response is an ERR line.
        Raises TimeoutError if no OK/ERR arrives within timeout.
        """
        timeout = timeout if timeout is not None else self._timeout
        with self._write_lock:
            self._cmd_in_flight.set()
            self._serial.write((cmd.strip() + "\n").encode("ascii"))
            self._serial.flush()

        try:
            lines    = []
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No OK/ERR response to '{cmd}' within {timeout}s; got: {lines}"
                    )
                try:
                    line = self._response_queue.get(timeout=remaining)
                except queue.Empty:
                    raise TimeoutError(
                        f"No OK/ERR response to '{cmd}' within {timeout}s; got: {lines}"
                    )
                lines.append(line)
                if line.startswith("OK") or line == "OK":
                    return lines
                if line.startswith("ERR") or line == "ERR":
                    raise LaserError(f"Firmware error: {line}")
        finally:
            self._cmd_in_flight.clear()


# ---------------------------------------------------------------------------
# LaserController — high-level API
# ---------------------------------------------------------------------------

class LaserController:
    """
    High-level interface to the Diamond C-20 laser controller firmware.

    Typical workflow:
        laser.set_power(0.5)
        laser.enable()
        laser.on()          # indefinite
        ...
        laser.off()
        laser.disable()
    """

    def __init__(self, port: LaserPort):
        self._port = port

    # --- State management ---------------------------------------------------

    def enable(self) -> None:
        """Assert the Control Enable pin."""
        self._port.send_command("ENABLE")

    def disable(self) -> None:
        """De-assert Control Enable and stop modulation."""
        self._port.send_command("DISABLE")

    def fault_reset(self) -> None:
        """
        Attempt to clear a fault. Raises LaserError if faults persist.
        After a successful reset, call enable() then on() to resume.
        """
        self._port.send_command("FAULT_RESET")

    # --- Power and operation ------------------------------------------------

    def set_power(self, fraction: float) -> None:
        """Set output power as a fraction of maximum (0.0 – 1.0)."""
        if not (0.0 <= fraction <= 1.0):
            raise ValueError(f"Power fraction must be 0.0–1.0, got {fraction!r}")
        self._port.send_command(f"POWER {fraction:.4f}")

    def on(self, duration_ms: int | None = None) -> None:
        """
        Start emission. If duration_ms is given, the firmware will auto-stop
        after that many milliseconds.
        """
        if duration_ms is not None:
            self._port.send_command(f"ON {int(duration_ms)}")
        else:
            self._port.send_command("ON")

    def off(self) -> None:
        """Stop modulation. Control Enable remains asserted."""
        self._port.send_command("OFF")

    # --- Query --------------------------------------------------------------

    def get_status(self) -> dict:
        """
        Query firmware status. Returns a dict with keys:
          state, power, laser_ok, temp_ok, voltage_ok, faults, freq, duty
        """
        lines = self._port.send_command("STATUS")
        result = {}
        for line in lines:
            if not line.startswith("STATUS "):
                continue
            parts = line.split(None, 2)   # ["STATUS", "KEY", "value"]
            if len(parts) < 3:
                continue
            key = parts[1].lower()
            val = parts[2]
            result[key] = val
        return result

    def get_ident(self) -> str:
        """Return the firmware identification string."""
        lines = self._port.send_command("IDENT")
        for line in lines:
            if line.startswith("OK IDENT"):
                return line[len("OK IDENT"):].strip()
        return ""

    def read_meter(self) -> float:
        """Return the PM100D analog-out voltage as measured by the Teensy ADC."""
        lines = self._port.send_command("METER")
        for line in lines:
            if line.startswith("OK METER"):
                return float(line[len("OK METER"):].strip())
        raise LaserError(f"Unexpected METER response: {lines}")

    def wait_for_ready(self, timeout: float = 90.0, poll_interval: float = 2.0) -> None:
        """
        Block until the laser reports STATE READY (pre-ionization complete).
        Raises TimeoutError if not ready within timeout seconds.
        Raises LaserFault if a fault is detected while waiting.
        """
        deadline = time.monotonic() + timeout
        print(f"Waiting for laser to finish pre-ionization (up to {timeout:.0f} s)...")
        while time.monotonic() < deadline:
            status = self.get_status()
            state  = status.get("state", "").upper()
            faults = status.get("faults", "NONE").upper()

            print(f"  [{time.strftime('%H:%M:%S')}] State: {state}  Faults: {faults}")

            if state == "READY" or state == "ENABLED" or state == "RUNNING":
                print("  Laser is ready.")
                return
            if state == "FAULT":
                raise LaserFault(f"Fault detected while waiting for ready: {faults}")

            time.sleep(poll_interval)

        raise TimeoutError(f"Laser did not reach READY within {timeout:.0f} s")


# FaultMonitor is no longer a separate class. Unsolicited message handling is
# built into LaserPort's reader thread. Register a callback with:
#   port.set_unsolicited_callback(fn)


# ---------------------------------------------------------------------------
# PowerMeter — converts PM100D analog-out volts to absorbed/delivered watts
# ---------------------------------------------------------------------------

class PowerMeter:
    """
    Translates the PM100D 0–2 V analog output (sampled by the Teensy ADC) into
    measured and delivered laser power.

    "Measured" = power hitting the PM100D (e.g. the reflected leg of an OD 1.0
    reflective ND).  "Delivered" = the fraction that goes downstream to the
    target, computed as measured * transmission_ratio.
    """

    def __init__(self,
                 full_scale_W:        float = PM100D_FULL_SCALE_W,
                 analog_fs_V:         float = PM100D_ANALOG_OUT_FS_V,
                 transmission_ratio:  float = TRANSMISSION_RATIO):
        self.full_scale_W       = full_scale_W
        self.analog_fs_V        = analog_fs_V
        self.transmission_ratio = transmission_ratio

    def power_measured_W(self, volts: float) -> float:
        return volts / self.analog_fs_V * self.full_scale_W

    def power_delivered_W(self, volts: float) -> float:
        return self.power_measured_W(volts) * self.transmission_ratio


# ---------------------------------------------------------------------------
# PIDController — simple PI(D) with rate limiting and conditional-integration
# ---------------------------------------------------------------------------

class PIDController:
    """
    Simple PID with two safety features:

      1. Rate limit:  |output - prev_output|  capped at max_step per call.
         Guards against the loop demanding a big setpoint jump when a gain
         is wrong or the measurement is noisy.

      2. Anti-windup: the integral state is only committed if the (rate-limited)
         output is not saturated against output_min/output_max. Prevents the
         I term from accumulating against a clamp.
    """

    def __init__(self,
                 kp:         float,
                 ki:         float,
                 kd:         float = 0.0,
                 max_step:   float = 0.02,
                 output_min: float = 0.0,
                 output_max: float = 1.0):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.max_step   = max_step
        self.output_min = output_min
        self.output_max = output_max
        self._integral    = 0.0
        self._last_error  = None
        self._last_output = None

    def reset(self, initial_output: float = 0.0) -> None:
        self._integral    = 0.0
        self._last_error  = None
        self._last_output = initial_output

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            return self._last_output if self._last_output is not None else 0.0

        error = setpoint - measurement
        tentative_integral = self._integral + error * dt
        d_term = ((error - self._last_error) / dt
                  if self._last_error is not None else 0.0)
        raw = (self.kp * error
               + self.ki * tentative_integral
               + self.kd * d_term)

        # Rate limit against the previous applied output.
        if self._last_output is not None:
            lo = self._last_output - self.max_step
            hi = self._last_output + self.max_step
            raw = max(lo, min(hi, raw))

        # Saturate at output bounds.
        clamped = max(self.output_min, min(self.output_max, raw))

        # Conditional integration: commit only if not saturated.  abs() is
        # there to swallow floating-point hair when raw lands exactly on
        # the clamp.
        if abs(clamped - raw) < 1e-12:
            self._integral = tentative_integral

        self._last_error  = error
        self._last_output = clamped
        return clamped


# ---------------------------------------------------------------------------
# PowerFeedback — background thread that closes the loop
# ---------------------------------------------------------------------------

class PowerFeedback(threading.Thread):
    """
    Periodically reads the PM100D voltage via the Teensy, converts it to
    delivered watts, runs a PID step, and pushes a new POWER fraction to the
    firmware.

    Lifecycle:
        fb = PowerFeedback(laser, meter, pid, target_power_W=2.0)
        fb.start()                          # thread runs; feedback OFF
        ...
        fb.enable(initial_power=0.05)       # start regulating
        ...
        fb.disable()                        # stop regulating; thread keeps polling
        fb.stop(); fb.join()                # terminate thread

    The thread also serves as a passive monitor — `latest_volts` and
    `latest_measured_W` are kept up to date even when feedback is disabled,
    so the host can log the meter without writing a separate poller.
    """

    def __init__(self,
                 laser:                  "LaserController",
                 meter:                  PowerMeter,
                 pid:                    PIDController,
                 target_power_W:         float,
                 update_interval_s:      float = FEEDBACK_UPDATE_S):
        super().__init__(daemon=True)
        self._laser                  = laser
        self._meter                  = meter
        self._pid                    = pid
        self._target_W               = target_power_W
        self._update_interval_s      = update_interval_s
        self._stop_evt               = threading.Event()
        self._enabled                = False
        self._lock                   = threading.Lock()
        self._last_step_t            = None

        # Shared state — read by the main thread; protected by _lock.
        self.latest_volts        = 0.0
        self.latest_measured_W   = 0.0
        self.latest_delivered_W  = 0.0
        self.latest_setpoint     = 0.0

    @property
    def meter(self) -> PowerMeter:
        return self._meter

    def set_target(self, power_W: float) -> None:
        with self._lock:
            self._target_W = power_W

    def enable(self, initial_power_fraction: float) -> None:
        """Begin regulating. initial_power_fraction is the POWER value already
        applied by the caller — the PID starts from there to avoid a jump."""
        with self._lock:
            self._pid.reset(initial_output=initial_power_fraction)
            self._last_step_t = None
            self._enabled = True

    def disable(self) -> None:
        with self._lock:
            self._enabled = False

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        while not self._stop_evt.wait(self._update_interval_s):
            try:
                volts = self._laser.read_meter()
            except (LaserError, TimeoutError) as exc:
                print(f"[feedback] meter read failed: {exc}", file=sys.stderr)
                continue

            measured_W  = self._meter.power_measured_W(volts)
            delivered_W = self._meter.power_delivered_W(volts)

            with self._lock:
                self.latest_volts       = volts
                self.latest_measured_W  = measured_W
                self.latest_delivered_W = delivered_W
                enabled                 = self._enabled
                target_W                = self._target_W

            # Skip the PID step if disabled.  We deliberately do NOT auto-disable
            # on a 0 V reading — a properly wired PM100D drives the analog out
            # to 0 V when no light is on the sensor, which is indistinguishable
            # from an unplugged BNC.  The caller is responsible for engaging
            # feedback only after the laser is actually producing light.
            if not enabled:
                with self._lock:
                    self._last_step_t = None
                continue

            now = time.monotonic()
            with self._lock:
                if self._last_step_t is None:
                    self._last_step_t = now
                    continue
                dt = now - self._last_step_t
                self._last_step_t = now

            new_power = self._pid.update(target_W, delivered_W, dt)

            try:
                self._laser.set_power(new_power)
                with self._lock:
                    self.latest_setpoint = new_power
            except LaserError as exc:
                print(f"[feedback] set_power failed: {exc}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Demo functions
# ---------------------------------------------------------------------------

def demo_startup_sequence(laser: LaserController) -> None:
    """
    Full startup sequence: identify firmware, wait for READY, arm, set power.
    Does NOT fire the laser — call demo_timed_burst() or demo_power_sweep() after.
    """
    print("\n=== Firmware identification ===")
    ident = laser.get_ident()
    print(f"  {ident}")

    print("\n=== Waiting for laser ready ===")
    laser.wait_for_ready(timeout=90.0, poll_interval=3.0)

    print("\n=== Arming laser (Control Enable) ===")
    laser.enable()
    print("  Control Enable asserted.")

    print("\n=== Initial status ===")
    status = laser.get_status()
    for k, v in status.items():
        print(f"  {k.upper():<15} {v}")


def demo_timed_burst(laser: LaserController, power: float, duration_ms: int) -> None:
    """Fire a single timed burst at the given power level, then confirm it stops."""
    print(f"\n=== Timed burst: power={power:.3f}, duration={duration_ms} ms ===")
    laser.set_power(power)
    laser.on(duration_ms=duration_ms)
    status = laser.get_status()
    freq   = status.get("freq", "--")
    duty   = status.get("duty", "--")
    try:
        pulse_us = float(duty) / float(freq) * 1e6
        pulse_str = f"{pulse_us:.2f} µs"
    except (ValueError, ZeroDivisionError):
        pulse_str = "--"
    print(f"  Calculated frequency:  {freq} Hz")
    print(f"  Calculated duty cycle: {duty}")
    print(f"  Calculated pulse width: {pulse_str}")
    print(f"  Firing for {duration_ms} ms...")

    # Poll until the firmware stops the laser automatically.
    wait_s = duration_ms / 1000.0 + 0.5
    time.sleep(wait_s)

    status = laser.get_status()
    state  = status.get("state", "?")
    print(f"  State after timeout: {state}")
    if state.upper() in ("ENABLED", "READY"):
        print("  Laser stopped automatically as expected.")
    else:
        print(f"  Unexpected state '{state}' — sending OFF.")
        laser.off()


def demo_power_sweep(laser: LaserController, steps: int = 10, dwell_ms: int = 500) -> None:
    """
    Sweep power from 0.1 to 1.0 in equal steps, dwelling at each for dwell_ms.
    Useful for verifying the oscilloscope shows the expected waveforms.
    """
    print(f"\n=== Power sweep: {steps} steps, {dwell_ms} ms each ===")
    power_levels = [round(0.1 + i * (0.9 / (steps - 1)), 4) for i in range(steps)]

    # Pre-query status at each step by sending POWER then ON.
    for p in power_levels:
        print(f"  Setting power {p:.4f}...")
        laser.set_power(p)
        laser.on()          # restart ON to apply new power live
        time.sleep(dwell_ms / 1000.0)

    laser.off()
    print("  Sweep complete.")


def demo_feedback_hold(laser: LaserController,
                       feedback: "PowerFeedback",
                       target_W: float,
                       hold_s:   float,
                       initial_power: float = 0.05,
                       warmup_s:      float = FEEDBACK_WARMUP_S) -> None:
    """Hold delivered power at target_W for hold_s seconds using the PID loop.

    A warm-up period is inserted between `laser.on()` and the PID engaging so the
    loop doesn't react to the laser's 0 W startup transient — the meter reads
    ~0 V for the first second or two while the RF settles and the discharge
    stabilises."""
    print(f"\n=== Feedback hold: target={target_W:.3f} W delivered, "
          f"duration={hold_s:.1f} s ===")
    print(f"  PM100D full scale:   {feedback.meter.full_scale_W:.1f} W")
    print(f"  Transmission ratio:  {feedback.meter.transmission_ratio:.4f}")
    print(f"  Initial POWER:       {initial_power:.4f}")
    print(f"  Warm-up:             {warmup_s:.1f} s")

    if not feedback.is_alive():
        feedback.start()

    # Start the laser at the conservative initial power and let it stabilise.
    laser.set_power(initial_power)
    laser.on()
    feedback.set_target(target_W)

    t_warm = time.monotonic()
    while time.monotonic() - t_warm < warmup_s:
        time.sleep(0.5)
        print(f"  [warmup t={time.monotonic()-t_warm:4.1f}s]  "
              f"V={feedback.latest_volts:.4f}  "
              f"P_meas={feedback.latest_measured_W:6.3f} W")

    feedback.enable(initial_power_fraction=initial_power)
    print("  Feedback engaged.")

    t0 = time.monotonic()
    next_print = t0
    try:
        while time.monotonic() - t0 < hold_s:
            time.sleep(0.1)
            if time.monotonic() >= next_print:
                print(f"  t={time.monotonic()-t0:5.1f}s  "
                      f"V={feedback.latest_volts:.4f}  "
                      f"P_meas={feedback.latest_measured_W:6.3f} W  "
                      f"P_deliv={feedback.latest_delivered_W:6.3f} W  "
                      f"POWER={feedback.latest_setpoint:.4f}")
                next_print += 1.0
    finally:
        feedback.disable()
        laser.off()
        print("  Hold complete; feedback disabled.")


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Diamond C-20 CO2 laser controller demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 2-second burst at 50% power
  python laser_controller.py --port /dev/ttyACM0 --power 0.5 --duration 2000

  # Sweep power from 10% to 100%
  python laser_controller.py --port COM3 --sweep

  # Low-power burst (1% — tests variable-frequency regime)
  python laser_controller.py --port /dev/ttyACM0 --power 0.01 --duration 1000

  # Hold 2 W delivered for 30 s using PM100D feedback (requires meter connected)
  python laser_controller.py --port /dev/ttyACM0 --feedback 2.0 --hold 30
        """,
    )
    parser.add_argument("--port",     required=True,             help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--power",    type=float, default=0.5,   help="Power fraction 0.0–1.0 (default 0.5)")
    parser.add_argument("--duration", type=int,   default=2000,  help="Burst duration in ms (default 2000)")
    parser.add_argument("--sweep",    action="store_true",        help="Run power sweep instead of single burst")
    parser.add_argument("--feedback", type=float, default=None,
                        help="Hold delivered power (in W) using PM100D analog-out feedback")
    parser.add_argument("--hold",     type=float, default=10.0,  help="Duration (s) for --feedback hold (default 10)")
    args = parser.parse_args()

    if not (0.0 < args.power <= 1.0):
        print("ERROR: --power must be between 0.0 (exclusive) and 1.0 (inclusive).")
        sys.exit(1)
    if args.feedback is not None and args.feedback <= 0.0:
        print("ERROR: --feedback target power must be > 0.")
        sys.exit(1)

    fault_lines = []

    def fault_callback(line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        msg = f"[{ts}] UNSOLICITED: {line}"
        print(msg, file=sys.stderr)
        fault_lines.append(line)

    try:
        with LaserPort(args.port) as port:
            port.set_unsolicited_callback(fault_callback)
            laser = LaserController(port)

            meter    = PowerMeter()
            pid      = PIDController(kp=FEEDBACK_KP, ki=FEEDBACK_KI, kd=FEEDBACK_KD,
                                     max_step=FEEDBACK_MAX_STEP)
            feedback = PowerFeedback(laser, meter, pid,
                                     target_power_W=args.feedback or 0.0)

            try:
                demo_startup_sequence(laser)

                if args.feedback is not None:
                    demo_feedback_hold(laser, feedback,
                                       target_W=args.feedback,
                                       hold_s=args.hold)
                elif args.sweep:
                    demo_power_sweep(laser)
                else:
                    demo_timed_burst(laser, args.power, args.duration)

            except (LaserFault, LaserError) as exc:
                print(f"\nLASER ERROR: {exc}", file=sys.stderr)
                sys.exit(1)

            finally:
                print("\n=== Shutdown ===")
                try:
                    feedback.disable()
                    feedback.stop()
                    if feedback.is_alive():
                        feedback.join(timeout=1.0)
                    laser.off()
                    laser.disable()
                    print("  Laser disarmed.")
                except Exception as exc:
                    print(f"  Shutdown error (ignored): {exc}", file=sys.stderr)

    except serial.SerialException as exc:
        print(f"Serial port error: {exc}", file=sys.stderr)
        sys.exit(1)

    if any(line.startswith("FAULT") for line in fault_lines):
        print(f"\nWARNING: {sum(1 for l in fault_lines if l.startswith('FAULT'))} fault message(s) received during session.")
        sys.exit(2)

    print("\nDone.")


if __name__ == "__main__":
    main()
