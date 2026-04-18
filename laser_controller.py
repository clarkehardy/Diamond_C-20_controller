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
    status = laser.get_status()
    freq   = status.get("freq",  "--")
    duty   = status.get("duty",  "--")
    print(f"  Calculated frequency: {freq} Hz")
    print(f"  Calculated duty cycle: {duty}")
    print(f"  Firing for {duration_ms} ms...")
    laser.on(duration_ms=duration_ms)

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
        """,
    )
    parser.add_argument("--port",     required=True,            help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--power",    type=float, default=0.5,  help="Power fraction 0.0–1.0 (default 0.5)")
    parser.add_argument("--duration", type=int,   default=2000, help="Burst duration in ms (default 2000)")
    parser.add_argument("--sweep",    action="store_true",       help="Run power sweep instead of single burst")
    args = parser.parse_args()

    if not (0.0 < args.power <= 1.0):
        print("ERROR: --power must be between 0.0 (exclusive) and 1.0 (inclusive).")
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

            try:
                demo_startup_sequence(laser)

                if args.sweep:
                    demo_power_sweep(laser)
                else:
                    demo_timed_burst(laser, args.power, args.duration)

            except (LaserFault, LaserError) as exc:
                print(f"\nLASER ERROR: {exc}", file=sys.stderr)
                sys.exit(1)

            finally:
                print("\n=== Shutdown ===")
                try:
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
