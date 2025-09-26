import math

import structlog
import socket
import struct
import time


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
            print(f"[client] waiting for telemetry TCP on {host}:{port}...")
            conn, _ = self.sock.accept()
            self.sock = conn
            print("[client] connected to udp_diff telemetry")
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

    def update(self):
        data = self.recv_telemetry()
        self.pose = data["pose"]
        self.velocity = data["velocity"]
        self.angular_velocity = data["angular_velocity"]
        self.lidar_ranges = data["lidar_ranges"]
        self.last_update_time = time.time()

    def close(self):
        self.sock.close()


class Robot:
    def __init__(self, commands: CommandClient, telemetry: TelemetryClient):
        self.state: str = "FORWARD"
        self.commands = commands

        self.telemetry = telemetry
        self.angle = 0

        self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.start_time = time.time()
        self.last_move_time = self.start_time

    def forward_to_the_wall(self):
        try:
            while True:
                self.telemetry.print()
                time.sleep(5)

        except KeyboardInterrupt:
            print("[client] stop")
            self.commands.send_command(0.0, 0.0)

    def turn_by_angle_right(self, delta_theta: float):
        last_time = time.time()
        target_angle = self.angle - delta_theta
        self.commands.send_command(0.0, -0.6)
        self.telemetry.update()
        while self.angle > target_angle:
            self.angle += self.telemetry.angular_velocity[1] * (time.time() - last_time)
            last_time = time.time()
            self.telemetry.update()
        self.commands.send_command(0.0, 0.0)

    def turn_by_angle(self, delta_theta: float):
        # delta_theta > 0 → влево, < 0 → вправо
        current_angle = 0.0
        last_time = time.time()
        target = abs(delta_theta)  # работаем с модулем
        direction = 1.0 if delta_theta >= 0 else -1.0

        while current_angle < target:
            self.telemetry.update()
            wy = self.telemetry.angular_velocity[1]  # твоя ось!
            dt = time.time() - last_time
            last_time = time.time()
            current_angle += abs(wy) * dt  # угол всегда растёт

            # === САМОЕ ВАЖНОЕ: СНИЖАЕМ СКОРОСТЬ ПРИ ПРИБЛИЖЕНИИ ===
            remaining = target - current_angle
            if remaining < 0.3:  # последние ~17 градусов
                speed = 0.2 * direction  # медленно
            elif remaining < 0.6:  # последние ~34 градуса
                speed = 0.4 * direction
            else:
                speed = 0.6 * direction  # полная скорость

            self.commands.send_command(0.0, speed)
            time.sleep(0.01)

        self.commands.send_command(0.0, 0.0)

    def forward(self):
        self.commands.send_command(0.5, 0.0)  # поворот влево
        time.sleep(20)
        self.commands.send_command(0.0, 0.0)


