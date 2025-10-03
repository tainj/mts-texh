import math
import time

from dotenv import load_dotenv
import os
from robot import Robot, CommandClient, TelemetryClient


# Загружаем из .env
load_dotenv()

CMD_HOST: str  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT: int  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST: str  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT: int = int(os.getenv("TEL_PORT", "5600"))
PROTO: str    = str(os.getenv("PROTO", "tcp"))


if __name__ == "__main__":
     # Инициализация робота
    commands = CommandClient(CMD_HOST, CMD_PORT)
    telemetry = TelemetryClient(TEL_HOST, TEL_PORT)
    bot = Robot(commands, telemetry)

    # Первый этап
    bot.telemetry.visualize_lidar_front()
    bot.turn_by_angle2(math.pi)
    bot.telemetry.visualize_lidar_front()
    bot.forward_to_the_wall()
    time.sleep(0.5)
    bot.telemetry.visualize_lidar_front()
    print("*************************************************************************************************************************************************")
    bot.turn_by_angle2(math.pi * 2 / 5)
    bot.telemetry.visualize_lidar_front()
