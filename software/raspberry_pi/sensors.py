"""Sensor abstractions used by the robot controller connected via Serial to Arduino."""

from dataclasses import dataclass
import serial
import threading
import time


@dataclass
class DistanceReadings:
    front_clear: bool = True
    front_distance_cm: float | None = None
    ir_left_clear: bool = True
    ir_right_clear: bool = True


class SimulatedDistanceSensors:
    """Kept for dry_run / bench testing without hardware attached."""

    def __init__(self) -> None:
        self.readings = DistanceReadings()

    def read(self) -> DistanceReadings:
        return self.readings

    def set_front_obstacle(self, distance_cm: float | None) -> None:
        self.readings = DistanceReadings(
            front_clear=distance_cm is None,
            front_distance_cm=distance_cm,
        )

    def close(self) -> None:
        pass


class ArduinoDistanceSensors:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200) -> None:
        self.readings = DistanceReadings()
        self.running = True

        try:
            # Arduino USB Baglantisi
            self.ser = serial.Serial(port, baudrate, timeout=1.0)

            # Arduino, port acilinca DTR tetiklemesiyle resetlenir.
            # Reset bitene kadar bekleyip yarim/bozuk satirlari temizliyoruz.
            time.sleep(2.0)
            self.ser.reset_input_buffer()

            # Arka planda surekli Arduino'yu dinleyecek bir thread baslatiyoruz
            self.thread = threading.Thread(target=self._listen_arduino, daemon=True)
            self.thread.start()
            print(f"Arduino connected on {port}")
        except Exception as e:
            print(f"Warning: Could not connect to Arduino on {port}: {e}")
            print("Running with fallback simulated sensor data.")
            self.ser = None

    def _listen_arduino(self) -> None:
        while self.running and self.ser:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    parts = line.split(',')
                    if len(parts) == 3:
                        # Arduino: ir1, ir2, distance
                        ir1 = int(parts[0])
                        ir2 = int(parts[1])
                        distance_cm = float(parts[2])

                        # E18-D80NK sensorler engel gorunce LOW (0) verir.
                        ir_left_clear = (ir1 == 1)
                        ir_right_clear = (ir2 == 1)

                        # Ultrasonik hata durumunda -1 basiyordu, onu None yapalim
                        actual_dist = None if distance_cm == -1 else distance_cm

                        # Eger ultrasonik 10cm altindaysa VEYA IR sensorlerden biri engel gorduyse onumuz kapalidir
                        front_clear = True
                        if actual_dist is not None and actual_dist <= 10.0:
                            front_clear = False
                        if not ir_left_clear or not ir_right_clear:
                            front_clear = False

                        self.readings = DistanceReadings(
                            front_clear=front_clear,
                            front_distance_cm=actual_dist,
                            ir_left_clear=ir_left_clear,
                            ir_right_clear=ir_right_clear
                        )
                except Exception:
                    pass  # Hatali satir gelirse es gec
            time.sleep(0.01)

    def read(self) -> DistanceReadings:
        return self.readings

    def close(self) -> None:
        self.running = False
        if self.ser:
            self.ser.close()