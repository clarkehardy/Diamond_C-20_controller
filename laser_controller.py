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

    Responses from the firmware always end with a line starting 'OK' or 'ERR'.
    send_command() collects all lines up to and including that terminator and
    returns them as a list.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 5.0):
        self._port_name = port
        self._baudrate  = baudrate
        self._timeout   = timeout
        self._serial: serial.Serial | None = None
        self._read_lock  = threading.Lock()
        self._write_lock = threading.Lock()

    def open(self) -> None:
        self._serial = serial.Serial(
            self._port_name,
            self._baudrate,
            timeout=0.1,           # short read timeout for background polling
        )
        time.sleep(2.0)            # wait for Teensy USB CDC to enumerate

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def readline(self, timeout: float | None = None) -> str | None:
        """Read one line. Returns None on timeout."""
        deadline = time.monotonic() + (timeout if timeout is not None else self._timeout)
        buf = b""
        with self._read_lock:
            while time.monotonic() < deadline:
                if self._serial is None or not self._serial.is_open:
                    return None
                chunk = self._serial.readline()
                if chunk:
                    buf += chunk
                    if buf.endswith(b"\n"):
                        return buf.decode("ascii", errors="replace").strip()
        return None  # timeout

    def send_command(self, cmd: str, timeout: float | None = None) -> list[str]:
        """
        Send a command and collect the full response (up to OK/ERR terminator).
        Returns a list of response lines.
        Raises LaserError if the response is an ERR line.
        Raises TimeoutError if no OK/ERR arrives within timeout.
        """
        timeout = timeout if timeout is not None else self._timeout
        with self._write_lock:
            self._serial.write((cmd.strip() + "\n").encode("ascii"))
            self._serial.flush()

        lines = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.readline(timeout=max(0.0, deadline - time.monotonic()))
            if line is None:
                break
            # Silently pass unsolicited FAULT/STATUS lines to caller via lines list.
            lines.append(line)
            if line.startswith("OK ") or line == "OK":
                return lines
            if line.startswith("ERR ") or line == "ERR":
                raise LaserError(f"Firmware error: {line}")

        raise TimeoutError(f"No OK/ERR response to '{cmd}' within {timeout}s; got: {lines}")


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


# ---------------------------------------------------------------------------
# FaultMonitor — background thread for unsolicited messages
# ---------------------------------------------------------------------------

class FaultMonitor(threading.Thread):
    """
    Background thread that reads lines from the firmware and calls a callback
    for any unsolicited FAULT or STATUS message.

    The main thread and this thread share the same serial port object.
    The LaserPort.readline() method uses a lock, so concurrent reads are safe,
    but note that send_command() also reads lines — there is a potential race.

    For production use, a single-reader architecture (all reads on one thread)
    with a queue is more robust. This simpler approach works for demonstration.
    """

    def __init__(self, port: LaserPort, callback):
        super().__init__(daemon=True)
        self._port     = port
        self._callback = callback
        self._stop_evt = threading.Event()

    def run(self) -> None:
        while not self._stop_evt.is_set():
            line = self._port.readline(timeout=0.2)
            if line is None:
                continue
            if line.startswith("FAULT") or line.startswith("STATUS"):
                try:
                    self._callback(line)
                except Exception as exc:
                    print(f"[FaultMonitor] callback raised: {exc}", file=sys.stderr)

    def stop(self) -> None:
        self._stop_evt.set()
        self.join(timeout=1.0)


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
            laser   = LaserController(port)
            monitor = FaultMonitor(port, fault_callback)
            monitor.start()

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

                monitor.stop()

    except serial.SerialException as exc:
        print(f"Serial port error: {exc}", file=sys.stderr)
        sys.exit(1)

    if any(line.startswith("FAULT") for line in fault_lines):
        print(f"\nWARNING: {sum(1 for l in fault_lines if l.startswith('FAULT'))} fault message(s) received during session.")
        sys.exit(2)

    print("\nDone.")


if __name__ == "__main__":
    main()
