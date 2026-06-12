"""
queue_send_test.py — Timeline-based send test (no GUI, no Q? polling).

How it works
------------
Instead of waiting for QFREE responses, we calculate exactly when each
command finishes executing on the Arduino and send the next one just before
it is needed.

    duration[i] = steps / feedrate          (feedrate is steps/sec in firmware)
                                             stepIntervalUs = 1_000_000 / feedrate

    execution_start[0] = t0
    execution_start[i] = execution_start[i-1] + duration[i-1]

The first ARDUINO_QUEUE commands are sent immediately to fill the hardware
queue.  Every subsequent command is sent when:

    now >= execution_start[i - ARDUINO_QUEUE + 1] - LOOKAHEAD_S

No serial response is needed — writes are non-blocking and the timing is
deterministic once the command duration is known.

Usage
-----
    python queue_send_test.py            # uses PREFERRED_PORT below
    python queue_send_test.py COM9
    python queue_send_test.py /dev/ttyUSB0
"""

import sys
import threading
import time

import serial

# ── tuneable constants ────────────────────────────────────────────────────────
PREFERRED_PORT  = "COM9"
BAUDRATE        = 115_200
SERIAL_TIMEOUT  = 0.1       # short so readline() doesn't stall the reader
CONNECT_DELAY   = 2.0       # wait for Arduino reset on DTR
ARDUINO_QUEUE   = 16        # must match COMMAND_QUEUE_SIZE in firmware
LOOKAHEAD_S     = 0.05      # send this many seconds before a slot is needed

N_COMMANDS      = 16        # 8 back-and-forth cycles × 2 commands each
STEPS_PER_CMD   = 400       # steps per command
FEEDRATE        = 4000      # steps/second  (firmware: stepIntervalUs = 1e6/feedrate)
# ─────────────────────────────────────────────────────────────────────────────


def build_command(steps: int, direction: int, feedrate: int) -> bytes:
    """steps,direction,feedrate\n — exact format the firmware expects."""
    return f"{steps},{direction},{feedrate}\n".encode("ascii")


def cmd_duration(steps: int, feedrate: int) -> float:
    return steps / feedrate if feedrate > 0 else 0.0


def _ts() -> str:
    return f"{time.monotonic():.3f}"


# ── minimal thread-safe serial wrapper ───────────────────────────────────────

class SerialSender:
    def __init__(self) -> None:
        self._conn: serial.Serial | None = None
        self._write_lock = threading.Lock()   # writes only — reads are lock-free
        self._running = False
        self._rx_thread: threading.Thread | None = None

    def connect(self, port: str) -> None:
        self._conn = serial.Serial(port, BAUDRATE, timeout=SERIAL_TIMEOUT)
        self._conn.reset_input_buffer()
        self._conn.reset_output_buffer()
        print(f"[{_ts()}] Opened {port}. Waiting {CONNECT_DELAY} s for Arduino boot …")
        time.sleep(CONNECT_DELAY)
        self._conn.reset_input_buffer()
        self._running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

    def disconnect(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        with self._write_lock:
            if self._conn and self._conn.is_open:
                self._conn.close()
        print(f"[{_ts()}] Port closed.")

    def send(self, steps: int, direction: int, feedrate: int) -> None:
        data = build_command(steps, direction, feedrate)
        with self._write_lock:
            self._conn.write(data)
            self._conn.flush()

    def _reader(self) -> None:
        """Background reader — never holds _write_lock during readline()."""
        while self._running:
            try:
                if not self._conn or not self._conn.is_open:
                    break
                raw = self._conn.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if line and not line.startswith("IMU,"):
                    print(f"  ← {line}")
            except Exception as exc:
                print(f"[reader error] {exc}")
                break


# ── main test ─────────────────────────────────────────────────────────────────

def run_test(port: str) -> None:
    commands = [
        (STEPS_PER_CMD, 1 if i % 2 == 0 else -1, FEEDRATE)
        for i in range(N_COMMANDS)
    ]
    total = len(commands)

    # Pre-compute relative execution timeline
    rel_start: list[float] = []
    t = 0.0
    for steps, _, feedrate in commands:
        rel_start.append(t)
        t += cmd_duration(steps, feedrate)
    total_duration = t

    sender = SerialSender()
    sender.connect(port)

    # Anchor to real time after connect delay
    t0 = time.monotonic()
    exec_start = [t0 + s for s in rel_start]
    exec_end   = t0 + total_duration

    dur_ms = cmd_duration(STEPS_PER_CMD, FEEDRATE) * 1000
    print(f"\n--- {total} commands, {total_duration:.2f} s total motion ---")
    print(f"    steps={STEPS_PER_CMD}  feedrate={FEEDRATE} steps/s  "
          f"duration/cmd={dur_ms:.1f} ms\n")

    # Phase 1: fill the queue immediately
    burst = min(total, ARDUINO_QUEUE)
    for i in range(burst):
        steps, direction, feedrate = commands[i]
        t_before = time.monotonic()
        sender.send(steps, direction, feedrate)
        write_us = (time.monotonic() - t_before) * 1e6
        print(f"  → [{i+1:3d}/{total}]  dir={direction:+d}  write={write_us:.0f} µs  (burst)")

    next_index = burst

    # Phase 2: timeline-paced sends
    while next_index < total:
        ref = max(0, next_index - ARDUINO_QUEUE + 1)
        send_at = exec_start[ref] - LOOKAHEAD_S

        wait = send_at - time.monotonic()
        if wait > 0.001:
            time.sleep(wait - 0.001)
        while time.monotonic() < send_at:
            pass   # tight spin for last ~1 ms

        steps, direction, feedrate = commands[next_index]
        t_before = time.monotonic()
        sender.send(steps, direction, feedrate)
        write_us = (time.monotonic() - t_before) * 1e6
        late_ms  = (time.monotonic() - send_at) * 1e3
        print(f"  → [{next_index+1:3d}/{total}]  dir={direction:+d}  "
              f"write={write_us:.0f} µs  late={late_ms:.1f} ms")
        next_index += 1

    print(f"\nAll {total} commands sent at t={time.monotonic()-t0:.3f} s")
    print(f"Waiting for motion to finish (expected {total_duration:.2f} s) …")
    while time.monotonic() < exec_end:
        time.sleep(0.05)

    print(f"\n=== Done in {time.monotonic()-t0:.3f} s (expected {total_duration:.2f} s) ===")
    sender.disconnect()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else PREFERRED_PORT
    try:
        run_test(port)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        sys.exit(1)
