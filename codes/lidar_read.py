import serial
import threading
import time
from typing import Optional


class LidarReader:
    """Read a YDLIDAR T-mini Plus over UART without the vendor SDK.

    Output is a list of measurements in degrees and millimeters:
    [
        {
            "angle": 123.45,
            "distance": 1000.0,
            "intensity": 50
        }
    ]
    """

    HEADER = b"\xAA\x55"

    # Standard YDLIDAR commands.
    CMD_PREFIX = 0xA5
    CMD_SCAN = 0x60
    CMD_STOP = 0x65
    CMD_SCAN_FREQUENCY_ADD_1HZ = 0x0B

    # Exclude the rear sector within 60 degrees of 180 degrees.
    REAR_EXCLUSION_START = 120.0
    REAR_EXCLUSION_END = 240.0

    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 230400,
        scan_frequency_increase_hz: int = 0
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.scan_frequency_increase_hz = max(
            0,
            min(int(scan_frequency_increase_hz), 6)
        )

        self.serial_port: Optional[serial.Serial] = None

        self._points: list[dict[str, float | int]] = []
        self._current_scan: list[dict[str, float | int]] = []

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._last_error: Optional[str] = None
        self._packet_count = 0
        self._checksum_error_count = 0
        self._scan_fps = 0.0
        self._last_scan_time: Optional[float] = None

    @property
    def running(self) -> bool:
        return self._running

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    @property
    def packet_count(self) -> int:
        return self._packet_count

    @property
    def checksum_error_count(self) -> int:
        return self._checksum_error_count

    def start(self) -> None:
        if self._running:
            return

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2
        )

        # Discard stale serial input.
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # While stopped, each 0xA5 0x0B command increases the T-mini Plus
        # scan frequency by 1 Hz, up to 12 Hz.
        for _ in range(self.scan_frequency_increase_hz):
            self._send_command(
                self.CMD_SCAN_FREQUENCY_ADD_1HZ
            )
            time.sleep(0.05)

        if self.scan_frequency_increase_hz:
            # Discard replies to the frequency commands before
            # receiving scan packets.
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()

        # Start scanning.
        self._send_command(self.CMD_SCAN)

        time.sleep(0.5)

        self._running = True

        self._thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name="tmini-plus-reader"
        )

        self._thread.start()

        print(
            f"LiDAR started: "
            f"{self.port} @ {self.baudrate}bps"
        )

    def stop(self) -> None:
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        if self.serial_port is not None:
            try:
                self._send_command(self.CMD_STOP)
                time.sleep(0.1)
            except Exception:
                pass

            self.serial_port.close()
            self.serial_port = None

        print("LiDAR stopped")

    def get_points(self) -> list[dict[str, float | int]]:
        """Copy the latest published scan under the reader lock."""
        with self._lock:
            return [point.copy() for point in self._points]

    def get_fps(self) -> float:
        with self._lock:
            return self._scan_fps

    def get_status(self) -> dict:
        return {
            "running": self.running,
            "port": self.port,
            "baudrate": self.baudrate,
            "scan_frequency_increase_hz": (
                self.scan_frequency_increase_hz
            ),
            "point_count": len(self.get_points()),
            "fps": round(self.get_fps(), 1),
            "packet_count": self.packet_count,
            "checksum_errors": self.checksum_error_count,
            "last_error": self.last_error
        }

    def _send_command(self, command: int) -> None:
        if self.serial_port is None:
            return

        packet = bytes([
            self.CMD_PREFIX,
            command
        ])

        self.serial_port.write(packet)
        self.serial_port.flush()

    def _read_exactly(self, size: int) -> Optional[bytes]:
        """Combine partial serial reads, returning None on timeout or shutdown."""
        if self.serial_port is None:
            return None

        data = bytearray()

        while len(data) < size and self._running:
            chunk = self.serial_port.read(
                size - len(data)
            )

            if not chunk:
                return None

            data.extend(chunk)

        if len(data) != size:
            return None

        return bytes(data)

    def _find_header(self) -> bool:
        """Read until the packet header bytes 0xAA 0x55 are found."""

        if self.serial_port is None:
            return False

        previous = None

        while self._running:
            current = self.serial_port.read(1)

            if not current:
                return False

            value = current[0]

            if previous == 0xAA and value == 0x55:
                return True

            previous = value

        return False

    def _read_loop(self) -> None:
        while self._running:
            try:
                if not self._find_header():
                    continue

                packet = self._read_packet()

                if packet is None:
                    continue

                self._packet_count += 1
                self._process_packet(packet)
                self._last_error = None

            except serial.SerialException as error:
                self._last_error = (
                    f"Serial error: {error}"
                )

                print(self._last_error)
                time.sleep(0.5)

            except Exception as error:
                self._last_error = str(error)

                print(
                    f"LiDAR parse error: {error}"
                )

                time.sleep(0.01)

    def _read_packet(self) -> Optional[dict]:
        """Read a packet using the T-mini Plus three-byte intensity sample format.

        Packet layout:
            PH  : 2 bytes, AA 55
            CT  : 1 byte
            LSN : 1 byte (sample count)
            FSA : 2 bytes (first sample angle)
            LSA : 2 bytes (last sample angle)
            CS  : 2 bytes (checksum)
            Si  : LSN * 3 bytes (intensity and distance samples)
        """

        fixed = self._read_exactly(8)

        if fixed is None:
            return None

        ct = fixed[0]
        lsn = fixed[1]

        if lsn == 0 or lsn > 100:
            return None

        fsa = int.from_bytes(
            fixed[2:4],
            byteorder="little"
        )

        lsa = int.from_bytes(
            fixed[4:6],
            byteorder="little"
        )

        received_checksum = int.from_bytes(
            fixed[6:8],
            byteorder="little"
        )

        sample_size = lsn * 3

        sample_data = self._read_exactly(
            sample_size
        )

        if sample_data is None:
            return None

        calculated_checksum = self._calculate_checksum(
            ct=ct,
            lsn=lsn,
            fsa=fsa,
            lsa=lsa,
            sample_data=sample_data
        )

        if calculated_checksum != received_checksum:
            self._checksum_error_count += 1

            return None

        return {
            "ct": ct,
            "lsn": lsn,
            "fsa": fsa,
            "lsa": lsa,
            "sample_data": sample_data
        }

    @staticmethod
    def _calculate_checksum(
        ct: int,
        lsn: int,
        fsa: int,
        lsa: int,
        sample_data: bytes
    ) -> int:
        """Calculate the checksum for three-byte samples containing intensity."""

        checksum = 0x55AA
        checksum ^= fsa

        for index in range(
            0,
            len(sample_data),
            3
        ):
            intensity_byte = sample_data[index]

            distance_word = (
                sample_data[index + 1]
                | sample_data[index + 2] << 8
            )

            checksum ^= intensity_byte
            checksum ^= distance_word

        checksum ^= (
            ct
            | lsn << 8
        )

        checksum ^= lsa

        return checksum & 0xFFFF

    def _process_packet(self, packet: dict) -> None:
        ct = packet["ct"]
        lsn = packet["lsn"]
        fsa = packet["fsa"]
        lsa = packet["lsa"]
        sample_data = packet["sample_data"]

        # CT bit 0 marks the first packet of a new revolution.
        is_start_packet = bool(ct & 0x01)

        if is_start_packet:
            if self._current_scan:
                completed_scan = self._clean_scan(
                    self._current_scan
                )

                if completed_scan:
                    now = time.monotonic()

                    with self._lock:
                        self._points = completed_scan

                        if self._last_scan_time is not None:
                            interval = now - self._last_scan_time

                            if interval > 0:
                                current_fps = 1.0 / interval
                                self._scan_fps = (
                                    current_fps
                                    if self._scan_fps == 0.0
                                    else self._scan_fps * 0.8
                                    + current_fps * 0.2
                                )

                        self._last_scan_time = now

            self._current_scan = []

        start_angle = (
            (fsa >> 1) / 64.0
        )

        end_angle = (
            (lsa >> 1) / 64.0
        )

        angle_difference = (
            end_angle - start_angle
        )

        if angle_difference < 0:
            angle_difference += 360.0

        for index in range(lsn):
            offset = index * 3

            intensity_low = sample_data[offset]
            distance_low = sample_data[offset + 1]
            distance_high = sample_data[offset + 2]

            raw_distance = (
                distance_low
                | distance_high << 8
            )

            # Decode the YDLIDAR sample format with intensity.
            intensity = (
                intensity_low
                | (distance_low & 0x03) << 8
            )

            # The upper 14 bits hold the distance.
            distance = raw_distance >> 2

            if lsn > 1:
                angle = (
                    start_angle
                    + angle_difference
                    * index
                    / (lsn - 1)
                )
            else:
                angle = start_angle

            angle %= 360.0

            # Discard points in the rear sector: 180 degrees +/- 60 degrees.
            if (
                self.REAR_EXCLUSION_START
                <= angle
                <= self.REAR_EXCLUSION_END
            ):
                continue

            # Mirror angles to account for the left-right reversal of the LiDAR mounting.
            angle = (-angle) % 360.0

            # Reject out-of-range distances.
            if distance < 50:
                continue

            if distance > 12000:
                continue

            self._current_scan.append({
                "angle": round(angle, 2),
                "distance": float(distance),
                "intensity": int(intensity)
            })

    @staticmethod
    def _clean_scan(
        points: list[dict[str, float | int]]
    ) -> list[dict[str, float | int]]:
        """Sort measurements by angle and remove near-duplicate readings."""

        valid_points = []

        for point in points:
            angle = float(point["angle"])
            distance = float(point["distance"])

            if not 0.0 <= angle < 360.0:
                continue

            if not 50.0 <= distance <= 12000.0:
                continue

            valid_points.append(point)

        valid_points.sort(
            key=lambda point: float(
                point["angle"]
            )
        )

        return valid_points
