import threading
import time
from typing import Callable, Optional


class TransportDisconnected(RuntimeError):
    pass


def _pyserial_factory(**kwargs):
    import serial

    return serial.Serial(**kwargs)


class SerialTransport:
    def __init__(
        self,
        port: str,
        baud_rate: int = 115200,
        timeout: float = 0.2,
        reconnect_interval: float = 1.0,
        on_line: Optional[Callable[[str], None]] = None,
        on_connection: Optional[Callable[[bool], None]] = None,
        serial_factory=None,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.reconnect_interval = reconnect_interval
        self.on_line = on_line
        self.on_connection = on_connection
        self._serial_factory = serial_factory or _pyserial_factory
        self._serial = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def _notify_connection(self, value: bool) -> None:
        if self.on_connection is not None:
            self.on_connection(value)

    def connect(self) -> None:
        if self.connected:
            return

        ser = self._serial_factory(
            port=self.port,
            baudrate=self.baud_rate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
        )
        with self._lock:
            self._serial = ser
        self._notify_connection(True)

    def _mark_disconnected(self) -> None:
        with self._lock:
            ser = self._serial
            self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self._notify_connection(False)

    def close(self) -> None:
        self._mark_disconnected()

    def read_line(self) -> str:
        with self._lock:
            ser = self._serial
        if ser is None or not bool(getattr(ser, "is_open", True)):
            self._mark_disconnected()
            raise TransportDisconnected("serial port is not connected")

        try:
            raw = ser.readline()
        except (OSError, IOError) as exc:
            self._mark_disconnected()
            raise TransportDisconnected(str(exc)) from exc

        if not raw:
            return ""
        if isinstance(raw, str):
            return raw.rstrip("\r\n")
        return raw.decode("utf-8", errors="replace").rstrip("\r\n")

    def write(self, payload: bytes) -> None:
        with self._lock:
            ser = self._serial
        if ser is None or not bool(getattr(ser, "is_open", True)):
            self._mark_disconnected()
            raise TransportDisconnected("serial port is not connected")

        try:
            ser.write(payload)
            ser.flush()
        except (OSError, IOError) as exc:
            self._mark_disconnected()
            raise TransportDisconnected(str(exc)) from exc

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker, name="roarm-serial", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self.close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.timeout + self.reconnect_interval + 0.5))
        self._thread = None

    def _worker(self) -> None:
        while not self._stop_event.is_set():
            if not self.connected:
                try:
                    self.connect()
                except (OSError, IOError):
                    self._stop_event.wait(self.reconnect_interval)
                    continue

            try:
                line = self.read_line()
            except TransportDisconnected:
                self._stop_event.wait(self.reconnect_interval)
                continue

            if line and self.on_line is not None:
                self.on_line(line)
            elif not line:
                time.sleep(0.001)
