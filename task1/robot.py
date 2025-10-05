import math
import logging
import structlog
import socket
import struct
import time

# Настройка логгера
structlog.configure(
    processors=[
        structlog.dev.ConsoleRenderer()
    ],
    wrapper_class=structlog.make_filtering_bound_logger(logging.DEBUG),
)

class CommandClient:
    """Отправка команд по UDP"""
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, v: float, w: float):
        packet = struct.pack("<2f", v, w)
        self.sock.sendto(packet, (self.host, self.port))


class TelemetryClient:
    """Приём телеметрии по TCP или UDP"""
    def __init__(self, host: str, port: int, proto: str = "tcp"):
        self.logger = structlog.getLogger("telemetry")
        self.host = host
        self.port = port
        self.proto = proto.lower()
        self.pose = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.angular_velocity = (0.0, 0.0, 0.0)
        self.lidar_ranges = []
        self.last_update_time = 0.0

        if self.proto == "tcp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.bind((host, port))
            self.sock.listen(1)
            self.logger.info("waiting fo telemetry TCP", host=host, port=port)
            conn, _ = self.sock.accept()
            self.sock = conn
            self.logger.info("connected to upd_diff telemetry")
        else:  # udp
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((host, port))  # или connect, зависит от архитектуры

    def recv_all(self, size: int) -> bytes:
        buf = b""
        while len(buf) < size:
            chunk = self.sock.recv(size - len(buf))
            if not chunk:
                raise ConnectionError("Соединение разорвано")
            buf += chunk
        return buf

    def recv_telemetry(self):
        if self.proto == "udp":
            data, _ = self.sock.recvfrom(65535)
        else:
            size_bytes = self.recv_all(4)
            size = struct.unpack("<I", size_bytes)[0]
            data = self.recv_all(size)

        if not data.startswith(b"WBTG"):
            raise ValueError("Неверная сигнатура пакета")

        header_size = 4 + 9 * 4  # 40 байт
        odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:header_size])
        n = struct.unpack("<I", data[header_size:header_size + 4])[0]
        ranges = []
        if n > 0:
            ranges = struct.unpack(f"<{n}f", data[header_size + 4:header_size + 4 + 4 * n])

        return {
            "pose": (odom_x, odom_y, odom_th),
            "velocity": (vx, vy, vth),
            "angular_velocity": (wx, wy, wz),
            "lidar_ranges": list(ranges)
        }

    def log(self):
        self.update()
        data = {
            "pose": tuple(round(x, 3) for x in self.pose),
            "velocity": tuple(round(v, 3) for v in self.velocity),
            "angular_velocity": tuple(round(v, 3) for v in self.angular_velocity),
            "lidar_ranges": [round(r, 2) for r in self.lidar_ranges[:10]]
        }
        self.logger.info("bot state", **data)

    def update(self, looking=False):
        data = self.recv_telemetry()
        self.pose = data["pose"]
        self.velocity = data["velocity"]
        self.angular_velocity = data["angular_velocity"]
        self.lidar_ranges = data["lidar_ranges"]
        self.last_update_time = time.time()
        if looking:
            self.logger.debug("update telemetry", time=self.last_update_time)

    def close(self):
        self.sock.close()

    def has_clear_path(self, threshold=0.55):
        for r in self.lidar_ranges:
            if 0.1 < r < 20.0:  # валидный диапазон
                if r < threshold:
                    print(self.lidar_ranges)
                    return False  # препятствие найдено
        return True  # путь чист

    def visualize_lidar_front(self):
        self.update()
        ranges = self.lidar_ranges
        if not ranges:
            print("Нет данных лидара")
            return

        n = len(ranges)
        if n != 360:
            print(f"Ожидалось 360 значений, получено {n}. Визуализация может быть неточной.")
            return

        result_vision = ""

        for i in range(0, len(ranges), 4):
            r = sum(ranges[i:i + 4]) / 4
            if r <= 0.3:
                result_vision += f"\033[91m{"█"}\033[0m"
            elif r <= 0.5:
                result_vision += f"\033[93m{"█"}\033[0m"
            elif r <= 1:
                result_vision += f"\033[92m{"█"}\033[0m"
            else:
                result_vision += " "
        self.logger.info("bot vision")
        print(result_vision)


class Robot:
    def __init__(self, commands: CommandClient, telemetry: TelemetryClient):
        self.logger = structlog.getLogger("bot")
        self.state: str = "FORWARD"
        self.commands = commands

        self.telemetry = telemetry
        self.angle = 0

        self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.start_time = time.time()
        self.last_move_time = self.start_time

    def turn_by_angle2(self, delta_theta: float):
        self.commands.send_command(0.0, 0.0)
        time.sleep(0.2)  # дать начать останавливаться

        # Ждём, пока скорость станет ПОЧТИ нулём
        while True:
            self.telemetry.update()
            self.commands.send_command(0.0, 0.0)
            vx, vy, vth = self.telemetry.velocity
            if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(vth) < 0.01:
                break
            time.sleep(0.02)

        # Теперь точно остановлены — начинаем поворот
        self.telemetry.update()
        start_th = self.telemetry.pose[2]
        target_th = start_th + delta_theta

        while True:
            self.telemetry.update()
            current_th = self.telemetry.pose[2]
            error = normalize_angle(target_th - current_th)

            if abs(error) < 0.08:  # ~5 градусов
                break

            if abs(error) > 0.3:
                w = 0.6 * (1 if error > 0 else -1)
            elif abs(error) > 0.1:
                w = 0.3 * (1 if error > 0 else -1)
            else:
                w = 0.12 * (1 if error > 0 else -1)

            self.commands.send_command(0.0, w)
            time.sleep(0.01)

        # Финальная остановка
        self.commands.send_command(0.0, 0.0)

    def forward_to_the_wall(self):
        self.logger.info("moving forward (acceleration-aware)")
        MAX_SPEED = 0.5  # ← безопасное значение даже при SPEEDUP=1
        TARGET_STOP = 0.4

        while True:
            self.telemetry.update()
            valid_ranges = [r for r in self.telemetry.lidar_ranges if 0.1 < r < 20.0]
            min_dist = min(valid_ranges) if valid_ranges else float('inf')

            if min_dist <= TARGET_STOP:
                speed = 0.0
            elif min_dist < 1.0:
                speed = MAX_SPEED * (min_dist - TARGET_STOP) / (1.0 - TARGET_STOP)
                speed = max(0.0, speed)
            else:
                speed = MAX_SPEED

            self.commands.send_command(speed, 0.0)
            if min_dist <= TARGET_STOP:
                break
            time.sleep(0.01)

        # Гарантированная остановка + учёт ускорения
        for _ in range(30):
            self.commands.send_command(0.0, 0.0)
            time.sleep(0.01)

        time.sleep(0.3)  # ← даём физике остановиться

def normalize_angle(angle):
    """Привести угол к диапазону [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))