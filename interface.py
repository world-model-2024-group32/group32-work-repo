import serial
import time
import numpy as np
import math

# チャンネル3は手首で500側が内側に閉じる、2500側が上に上がる
# チャンネル2は関節で500側が上向きに上げる
# チャンネル1
# チャンネル0はベースの旋回2500側で右に回り、500側で左に回る
# 500から2500
BASE_ANGLE = 1500
MAX_ANGLE = 2500
MIN_ANGLE = 500
TOTAL_CHANNELS = 6  # チャンネル数
BASE_SPEED = 300
LINK1_LENGTH = 145  # mm
LINK2_LENGTH = 130  # mm
LINK3_LENGTH = 60  # mm


class AL5D:
    def __init__(self, port="/dev/tty.usbserial-AB0K6DQX", baudrate=9600):
        self.base_angle = BASE_ANGLE
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.angles = [BASE_ANGLE] * TOTAL_CHANNELS  # 各チャンネルの角度を保持
        self.connect()
        self.initialize_position()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print("シリアルポートが開かれました。")
        except serial.serialutil.SerialException as e:
            print(f"シリアルポートエラー: {e}")

    def initialize_position(self):
        for channel in range(TOTAL_CHANNELS):
            self.move_servo(channel, self.base_angle, BASE_SPEED)
        print("すべてのサーボモーターを初期位置にセットしました。")

    def send_command(self, channel, position, speed):
        if self.ser and self.ser.is_open:
            command = f"#{channel} P{position} S{speed}\r"
            print(f"送信コマンド: {command}")
            self.ser.write(command.encode("ascii"))
            time.sleep(0.1)  # 各コマンドの間に少し待つ
            response = self.ser.read_all().decode("ascii")
            print(f"受信応答: {response}")

    def move_servo(self, channel, position, speed):
        if MIN_ANGLE <= position <= MAX_ANGLE:
            self.angles[channel] = position  # 角度を更新
            self.send_command(channel, position, speed)
        else:
            print(f"無効な位置: {position}。範囲は{MIN_ANGLE}から{MAX_ANGLE}です。")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("シリアルポートが閉じられました。")

    def move_incremental(self, dx: float, dy: float, dz: float):
        # 現在の位置を取得（仮のデータ）
        current_x, current_y, current_z = 0.0, 0.0, 0.0

        # 新しい目標位置を計算
        target_x = current_x + dx
        target_y = current_y + dy
        target_z = current_z + dz

        # 逆運動学を使用してサーボ角度を計算
        angles = self.inverse_kinematics(target_x, target_y, target_z)

        # 各サーボを移動
        for i, angle in enumerate(angles):
            self.move_servo(i, angle, BASE_SPEED)

    def forward_kinematics(self, theta1, theta2, theta3, theta4):
        x1 = LINK1_LENGTH * (math.cos(theta1) * math.cos(theta2))
        y1 = LINK1_LENGTH * (math.sin(theta1) * math.cos(theta2))
        z1 = LINK1_LENGTH * math.sin(theta2)
        x2 = x1 + (LINK2_LENGTH * (math.cos(theta1) * math.cos(theta2 + theta3)))
        y2 = x1 + (LINK2_LENGTH * (math.sin(theta1) * math.cos(theta2 + theta3)))
        z2 = x1 + (LINK2_LENGTH * math.sin(theta2 + theta3))
        x3 = x2 + (
            LINK3_LENGTH * (math.cos(theta1) * math.cos(theta2 + theta3 + theta4))
        )
        y3 = x2 + (
            LINK3_LENGTH * (math.sin(theta1) * math.cos(theta2 + theta3 + theta4))
        )
        z3 = x3 + ((LINK3_LENGTH * math.sin(theta2 + theta3 + theta4)))
        position = [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
        return position

    def inverse_kinematics(self, x: float, y: float, z: float) -> np.ndarray:
        theta1 = math.atan2()


if __name__ == "__main__":
    robot_arm = AL5D()
    # robot_arm.move_servo(0, 1700, 300)
    # robot_arm.move_servo(1, 2000, 300)
    # robot_arm.move_servo(2, 1500, 300)
    # robot_arm.move_servo(3, 1200, 300)
    robot_arm.move_servo(0, 2500, 300)
    print(robot_arm.angles)
    robot_arm.close()
