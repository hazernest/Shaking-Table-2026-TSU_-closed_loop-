import queue
import threading
import time

import serial
from serial.tools import list_ports

from .config import DEFAULT_BAUDRATE


def available_ports():
    return [f"{p.device} - {p.description}" for p in list_ports.comports()]


def port_device(label):
    return label.split(" - ", 1)[0].strip()


class SerialClient:
    def __init__(self, name, baudrate=DEFAULT_BAUDRATE):
        self.name = name
        self.baudrate = baudrate
        self.serial = None
        self.rx_queue = queue.Queue()
        self.log_queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._ok_event = threading.Event()

    def connect(self, port_label):
        self.disconnect()
        device = port_device(port_label)
        self.serial = serial.Serial(device, self.baudrate, timeout=0.05, write_timeout=1.0)
        time.sleep(1.8)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.log_queue.put(f"{self.name}: connected to {device}")

    def disconnect(self):
        self._stop.set()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None

    def is_connected(self):
        return bool(self.serial and self.serial.is_open)

    def send_line(self, line):
        if not self.is_connected():
            raise RuntimeError(f"{self.name} is not connected")
        self.serial.write((line.rstrip() + "\n").encode("ascii"))
        self.serial.flush()

    def send_motion_wait_ok(self, steps, direction, feedrate, timeout_s=10.0):
        self._ok_event.clear()
        self.send_line(f"{int(steps)},{int(direction)},{int(feedrate)}")
        if not self._ok_event.wait(timeout_s):
            raise TimeoutError(f"{self.name}: no ok after command")

    def _read_loop(self):
        while not self._stop.is_set() and self.serial and self.serial.is_open:
            try:
                raw = self.serial.readline()
            except Exception as exc:
                self.log_queue.put(f"{self.name}: read error: {exc}")
                break
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if text == "ok":
                self._ok_event.set()
            self.rx_queue.put(text)
