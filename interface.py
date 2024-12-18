import serial
import time
import numpy as np

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
LINK3_LENHTH = 60  # mm


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

    # def forward_kinematics(self):

    def inverse_kinematics(self, x: float, y: float, z: float) -> np.ndarray:
        # 簡単な逆運動学の例
        # 実際の計算はAL5Dの物理的な特性に依存します
        base_angle = int(BASE_ANGLE + (x * 100))
        shoulder_angle = int(BASE_ANGLE + (y * 100))
        elbow_angle = int(BASE_ANGLE + (z * 100))
        return np.array(
            [base_angle, shoulder_angle, elbow_angle, BASE_ANGLE, BASE_ANGLE]
        )


if __name__ == "__main__":
    robot_arm = AL5D()
    # robot_arm.move_servo(0, 1700, 300)
    # robot_arm.move_servo(1, 2000, 300)
    # robot_arm.move_servo(2, 1500, 300)
    # robot_arm.move_servo(3, 1200, 300)
    robot_arm.move_servo(0, 2500, 300)
    print(robot_arm.angles)
    robot_arm.close()
