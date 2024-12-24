import serial
import time
import numpy as np
import math

# チャンネル3は手首で500側が内側に閉じる、2500側が上に上がる
# チャンネル2は関節で500側が上向きに上げる
# チャンネル1
# チャンネル0はベースの旋回2500側で右に回り、500側で左に回る
# 500から2500
BASE_ANGLE = 0
MAX_ANGLE = 90
MIN_ANGLE = -90
TOTAL_CHANNELS = 6  # チャンネル数
BASE_SPEED = 300
LINK1_LENGTH = 145  # mm
LINK2_LENGTH = 130  # mm
LINK3_LENGTH = 60  # mm
THETA4 = math.radians(8)


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
            if channel == 0 or channel == 2:
                position *= -1  # 角度の向きを揃える
            # 角度を500msから2500msにマッピング
            # -90度が500ms、90度が2500msになるように変換
            position_ms = (position - MIN_ANGLE) * (2500 - 500) / (
                MAX_ANGLE - MIN_ANGLE
            ) + 500
            self.send_command(channel, int(position_ms), speed)
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
        print(f"θ1:{theta1},θ2:{theta2},θ3:{theta3}")
        x1 = LINK1_LENGTH * (math.cos(theta1) * math.cos(theta2))
        y1 = LINK1_LENGTH * (math.sin(theta1) * math.cos(theta2))
        z1 = LINK1_LENGTH * math.sin(theta2)
        x2 = x1 + (LINK2_LENGTH * (math.cos(theta1) * math.cos(theta2 + theta3)))
        y2 = y1 + (LINK2_LENGTH * (math.sin(theta1) * math.cos(theta2 + theta3)))
        z2 = z1 + (LINK2_LENGTH * math.sin(theta2 + theta3))
        x3 = x2 + (
            LINK3_LENGTH * (math.cos(theta1) * math.cos(theta2 + theta3 + theta4))
        )
        y3 = y2 + (
            LINK3_LENGTH * (math.sin(theta1) * math.cos(theta2 + theta3 + theta4))
        )
        z3 = z2 + ((LINK3_LENGTH * math.sin(theta2 + theta3 + theta4)))
        position = {"x": x3, "y": y3, "z": z3}
        return position

    def inverse_kinematics(self, x: float, y: float, z: float):
        # リンクの長さ
        L1 = LINK1_LENGTH
        L2 = LINK2_LENGTH
        L3 = LINK3_LENGTH

        # θ1の計算
        theta1 = math.atan2(y, x)

        # ベース座標系から見た腕の平面上の距離 r
        r = math.sqrt(x**2 + y**2)

        # リンク1を考慮しない高さ
        z_offset = z - LINK1_LENGTH

        # リンク1の高さを考慮しないベース座標から入力座標までの距離d
        d = math.sqrt(r**2 + z_offset**2)

        # dがリンク2とリンク3の長さの合計を超えていないか確認
        if d > (L2 + L3):
            raise ValueError("目標位置が到達不可能です。")

        diff_x = (
            L3
            * math.cos(self.angles[1] + self.angles[2] + self.angles[3])
            * math.cos(self.angles[0])
        )
        diff_y = (
            L3
            * math.cos(self.angles[1] + self.angles[2] + self.angles[3])
            * math.sin(self.angles[0])
        )
        diff_z = L3 * math.sin(self.angles[1] + self.angles[2] + self.angles[3])
        x2 = x - diff_x
        y2 = y - diff_y
        z2 = z - diff_z
        # θ3の計算（余弦定理）
        cos_theta3 = (x2**2 + y2**2 + z2**2 - (L1**2) - L2**2) / (2 * L1 * L2)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # 数値誤差の補正
        theta3_options = [
            math.acos(cos_theta3),
            -math.acos(cos_theta3),
        ]  # エルボーアップとエルボーダウン
        valid_solutions = []

        for theta3 in theta3_options:
            # θ2の計算
            beta = math.atan2(z2, math.sqrt(x2**2 + y2**2))
            gamma = math.acos(
                (x2**2 + y2**2 + z2**2 + L1**2 - L2**2)
                / (2 * L1 * math.sqrt(x2**2 + y2**2 + z2**2))
            )
            theta2_options = [beta + gamma, beta - gamma]

            for theta2 in theta2_options:
                # θ4は固定値
                theta4 = THETA4

                # ラジアンから度に変換
                theta1_deg = math.degrees(theta1)
                theta2_deg = math.degrees(theta2)
                theta3_deg = math.degrees(theta3)
                theta4_deg = math.degrees(theta4)

                # 角度を -180 度から 180 度の範囲に正規化
                theta1_deg = (theta1_deg + 180) % 360 - 180
                theta2_deg = (theta2_deg + 180) % 360 - 180
                theta3_deg = (theta3_deg + 180) % 360 - 180

                # 角度の範囲チェック
                if (
                    MIN_ANGLE <= theta1_deg <= MAX_ANGLE
                    and MIN_ANGLE <= theta2_deg <= MAX_ANGLE
                    and MIN_ANGLE <= theta3_deg <= MAX_ANGLE
                ):
                    # エルボーアップ/ダウンの識別
                    configuration = "Elbow Up" if theta3_deg < 0 else "Elbow Down"

                    # 有効な解を追加
                    valid_solutions.append(
                        {
                            "angles": [theta1_deg, theta2_deg, theta3_deg, theta4_deg],
                            "configuration": configuration,
                        }
                    )

        if not valid_solutions:
            raise ValueError("有効な逆運動学の解が見つかりませんでした。")

        # 順運動学との差が最も少ない解を選択
        best_solution = None
        min_difference = float("inf")
        for solution in valid_solutions:
            theta1_deg, theta2_deg, theta3_deg, theta4_deg = solution["angles"]
            fk_result = self.forward_kinematics(
                theta1_deg, theta2_deg, theta3_deg, theta4_deg
            )
            x_fk, y_fk, z_fk = fk_result["x"], fk_result["y"], fk_result["z"]
            difference = abs(x_fk - x) + abs(y_fk - y) + abs(z_fk - z)
            if difference < min_difference:
                min_difference = difference
                best_solution = solution

        return best_solution


if __name__ == "__main__":
    # 60.00, 34.00, 278.00
    robot_arm = AL5D()
    print(
        robot_arm.forward_kinematics(
            math.radians(robot_arm.angles[0]),
            math.radians(robot_arm.angles[1]),
            math.radians(robot_arm.angles[2]),
            math.radians(0),
        )
    )
    move_angles = robot_arm.inverse_kinematics(105, 75, 255)
    print(f"move_anglesはこちら{move_angles}")

    robot_arm.move_servo(0, move_angles["angles"][0], BASE_SPEED)
    robot_arm.move_servo(1, move_angles["angles"][1], BASE_SPEED)
    robot_arm.move_servo(2, move_angles["angles"][2], BASE_SPEED)
    robot_arm.move_servo(3, move_angles["angles"][3], BASE_SPEED)
    print(
        robot_arm.forward_kinematics(
            math.radians(robot_arm.angles[0]),
            math.radians(robot_arm.angles[1]),
            math.radians(robot_arm.angles[2]),
            THETA4,
        )
    )
    print(robot_arm.angles)
    robot_arm.close()
