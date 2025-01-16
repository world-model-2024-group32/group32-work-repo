import serial
import time
import numpy as np
import math
import threading

# import cv2
from typing import Dict, Any, Optional, Tuple

# チャンネル設定
BASE_ANGLE = 0
MAX_ANGLE = 90
MIN_ANGLE = -90
TOTAL_CHANNELS = 6  # チャンネル数
BASE_SPEED = 500
LINK1_LENGTH = 145  # mm
LINK2_LENGTH = 130  # mm
LINK3_LENGTH = 60  # mm
THETA4 = math.radians(8)
JOINT_NUM = 3

joint_range = [
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント0
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント1
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント2
]


# サイドの定義
# class Side(enum.Enum):
#     LEFT = 1
#     RIGHT = 2
#     OTHER = 3


class AL5D:
    def __init__(self, port="/dev/tty.usbserial-AB0K6DQX", baudrate=9600):
        self.ser_lock = threading.Lock()
        self.base_angle = BASE_ANGLE
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.angles = [BASE_ANGLE] * TOTAL_CHANNELS  # 各チャンネルの角度を保持
        self.connect()
        self.initialize_position()
        print("AL5Dクラスが初期化されました。")

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

    def send_command_with_time(self, channel, position, speed, move_time):
        if self.ser and self.ser.is_open:
            command = f"#{channel} P{position} S{speed} T{move_time}\r"
            print(f"送信コマンド: {command}")
            self.ser.write(command.encode("ascii"))
            time.sleep(0.1)  # 各コマンドの間に少し待つ
            response = self.ser.read_all().decode("ascii")
            print(f"受信応答: {response}")

    def move_servo(self, channel, position, speed):
        if MIN_ANGLE <= position <= MAX_ANGLE:
            self.angles[channel] = position  # 角度を更新
            if channel == 0 or channel == 1 or channel == 2:
                position *= -1  # 角度の向きを揃える
            # 角度を500msから2500msにマッピング
            # -90度が500ms、90度が2500msになるように変換
            position_ms = (position - MIN_ANGLE) * (2500 - 500) / (
                MAX_ANGLE - MIN_ANGLE
            ) + 500
            command = f"#{channel} P{int(position_ms)} S{speed}\r"
            self.send_command_and_wait(command)
        else:
            print(f"無効な位置: {position}。範囲は{MIN_ANGLE}から{MAX_ANGLE}です。")

    def move_servo_time(self, channel, position, speed, move_time):
        if MIN_ANGLE <= position <= MAX_ANGLE:
            self.angles[channel] = position  # 角度を更新
            if channel == 0 or channel == 2:
                position *= -1  # 角度の向きを揃える
            # 角度を500msから2500msにマッピング
            # -90度が500ms、90度が2500msになるように変換
            position_ms = (position - MIN_ANGLE) * (2500 - 500) / (
                MAX_ANGLE - MIN_ANGLE
            ) + 500
            command = f"#{channel} P{int(position_ms)} S{speed} T{move_time}\r"
            self.send_command_and_wait(command)
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
        for i, angle in enumerate(angles["angles"]):
            self.move_servo(i, angle, BASE_SPEED)

    def forward_kinematics(self, theta):
        L2 = LINK1_LENGTH
        L3 = LINK2_LENGTH

        S1 = math.sin(theta[0])
        C1 = math.cos(theta[0])
        S2 = math.sin(theta[1])
        C2 = math.cos(theta[1])
        S23 = math.sin(theta[1] + theta[2])
        C23 = math.cos(theta[1] + theta[2])

        x = C1 * (L2 * C2 + L3 * C23)
        y = S1 * (L2 * C2 + L3 * C23)
        z = L2 * S2 + L3 * S23

        return [x, y, z]

    def inverse_kinematics(self, x: float, y: float, z: float):
        L2 = LINK1_LENGTH
        L3 = LINK2_LENGTH

        theta = [0.0 for _ in range(JOINT_NUM)]

        # 手先位置の値が可動範囲の外であればエラー値を返す
        if (math.pow(L2 - L3, 2) > (x**2 + y**2 + z**2)) or (
            (x**2 + y**2 + z**2) > math.pow(L2 + L3, 2)
        ):
            print("目標位置が可動範囲外です。")
            return None

        # 1軸目theta[0]と2軸目theta[1],4軸目theta[3]以外は0 radで固定
        for i in range(JOINT_NUM):
            theta[i] = 0.0

        # 1軸目の角度
        theta[0] = math.atan2(y, x)

        # 3軸目の角度
        C3 = (x**2 + y**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)
        theta[2] = -math.acos(C3)  # acosは0:π/2の範囲で計算、θ3の可動はマイナス方向のみ
        # 2軸目の角度
        S3 = math.sin(theta[2])
        C2 = (L2 + L3 * C3) * math.sqrt(x**2 + y**2) + (L3 * S3) * z
        S2 = -(L3 * S3) * math.sqrt(x**2 + y**2) + (L2 + L3 * C3) * z
        theta[1] = math.atan2(S2, C2)

        # 得られた関節角度が可動範囲外であればエラーを返す
        for i in range(JOINT_NUM):
            if theta[i] < joint_range[i]["min"] or theta[i] > joint_range[i]["max"]:
                print(f"Theta[{i}] が可動範囲外です。")
                return None
        print(math.degrees(theta[0]), math.degrees(theta[1]), math.degrees(theta[2]))
        return theta

    def pulse_width_to_angle(self, pulse_width):
        """
        パルス幅（μs）を角度（度）に変換します。
        -90度が500μs、90度が2500μsに対応しています。
        """
        MIN_PULSE = 500
        MAX_PULSE = 2500
        MIN_ANGLE = -90
        MAX_ANGLE = 90
        angle = (
            (pulse_width - MIN_PULSE)
            * (MAX_ANGLE - MIN_ANGLE)
            / (MAX_PULSE - MIN_PULSE)
        ) + MIN_ANGLE
        return angle

    def get_servo_angle(self, channel):
        """
        指定されたサーボチャンネルの現在の角度を取得します。

        Args:
            channel (int): サーボのチャンネル番号（0から5）

        Returns:
            float: サーボの現在の角度（度）、取得できない場合は None
        """
        if not (0 <= channel < TOTAL_CHANNELS):
            print(
                f"無効なチャンネル番号: {channel}. 0から{TOTAL_CHANNELS - 1}の範囲で指定してください。"
            )
            return None

        try:
            command = f"QP {channel}\r"
            self.ser.write(command.encode("ascii"))
            # レスポンスの待機時間を確保（最大5ms）
            time.sleep(0.005)
            response = self.ser.read(1)  # サーボごとに1バイトのレスポンス

            if not response:
                print("応答がありません。")
                return None

            pulse_width_byte = response[0]
            pulse_width = pulse_width_byte * 10  # 10μsの解像度
            # angle = self.pulse_width_to_angle(pulse_width)
            return pulse_width

        except serial.SerialException as e:
            print(f"シリアル通信エラー: {e}")
            return None

    def query_movement_status(self):
        """
        モーターの動作状態を確認します。

        Returns:
            str: '.'（完了）または '+'（進行中）
        """
        try:
            with self.ser_lock:
                command = "Q\r"
                self.ser.write(command.encode("ascii"))
                # レスポンスの待機時間（最大5ms）
                time.sleep(0.005)
                response = self.ser.read(1).decode("ascii")
                return response
        except serial.SerialException as e:
            print(f"シリアル通信エラー: {e}")
            return None

    def wait_for_move_complete(self, timeout=5):
        """
        モーターの動作が完了するまで待機します。

        Args:
            timeout (int): 最大待機時間（秒）

        Returns:
            bool: 動作が完了した場合は True、タイムアウトした場合は False
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.query_movement_status()
            if status == ".":
                return True
            elif status == "+":
                time.sleep(0.1)  # 100ms待機して再確認
            else:
                print("不明な応答を受信しました。")
                return False
        print("動作がタイムアウトしました。")
        return False

    def send_command_and_wait(self, command):
        """
        コマンドを送信し、動作が完了するまで待機します。

        Args:
            command (str): 送信するコマンド文字列
        """
        try:
            with self.ser_lock:
                self.ser.write(command.encode("ascii"))
            if not self.wait_for_move_complete():
                print("動作の完了を待機中に問題が発生しました。")
        except serial.SerialException as e:
            print(f"シリアル通信エラー: {e}")

    def get_robot_state(self) -> Dict[str, Any]:
        """
        ロボットの現在の状態を取得します。

        Returns:
            Dict[str, Any]: ロボットの状態情報
        """
        state = {
            "base_angle": self.angles[0],
            "joint_angles": self.angles[1:4],
            "end_effector_pos": self.forward_kinematics(
                math.radians(self.angles[0]),
                math.radians(self.angles[1]),
                math.radians(self.angles[2]),
                math.radians(self.angles[3]),
            ),
            "gripper_pos": self.angles[5],
        }
        return state

    def move_coordinate(self, x, y, z):
        theta = self.inverse_kinematics(x, y, z)
        theta = [math.degrees(angle) for angle in theta]
        for i, angle in enumerate(theta):
            self.move_servo(i, angle, BASE_SPEED)


# class Environment:
#     def __init__(self):
#         self.robot = AL5D()
#         self.camera = cv2.VideoCapture(0)  # PCのカメラを使用
#         if not self.camera.isOpened():
#             raise Exception("カメラを開けませんでした。")
#         self.grasped_object = False
#         self.grasped_bin = Side.OTHER

#     def reset(self) -> Dict[str, Any]:
#         self.robot.initialize_position()
#         self.grasped_object = False
#         self.grasped_bin = Side.OTHER
#         ret, frame = self.camera.read()
#         if not ret:
#             raise Exception("カメラからの画像取得に失敗しました。")
#         obs = {
#             "base_angle": self.robot.angles[0],
#             "joint_angles": self.robot.angles[1:4],
#             "end_effector_pos": self.robot.get_robot_state()["end_effector_pos"],
#             "gripper_pos": self.robot.get_robot_state()["gripper_pos"],
#             "image": frame,
#             "reward": 0.0,
#             "is_first": True,
#             "is_last": False,
#             "is_terminal": False,
#         }
#         return obs

#     def step(self, action: Dict[str, Any]) -> Dict[str, Any]:
#         # 行動の実行
#         if action["action"] == "move_x_positive":
#             self.robot.move_incremental(10, 0, 0)
#         elif action["action"] == "move_x_negative":
#             self.robot.move_incremental(-10, 0, 0)
#         elif action["action"] == "move_y_positive":
#             self.robot.move_incremental(0, 10, 0)
#         elif action["action"] == "move_y_negative":
#             self.robot.move_incremental(0, -10, 0)
#         elif action["action"] == "move_z_positive":
#             self.robot.move_incremental(0, 0, 10)
#         elif action["action"] == "move_z_negative":
#             self.robot.move_incremental(0, 0, -10)
#         elif action["action"] == "gripper_open":
#             self.robot.move_servo(5, AL5D.GRIPPER_OPEN, BASE_SPEED)
#         elif action["action"] == "gripper_close":
#             self.robot.move_servo(5, AL5D.GRIPPER_CLOSE, BASE_SPEED)
#         else:
#             print(f"不明なアクション: {action['action']}")

#         time.sleep(0.5)  # アクション完了まで待機

#         # 観測の取得
#         ret, frame = self.camera.read()
#         if not ret:
#             raise Exception("カメラからの画像取得に失敗しました。")

#         state = self.robot.get_robot_state()
#         obs = {
#             "base_angle": self.robot.angles[0],
#             "joint_angles": self.robot.angles[1:4],
#             "end_effector_pos": state["end_effector_pos"],
#             "gripper_pos": state["gripper_pos"],
#             "image": frame,
#             "reward": self.compute_reward(),
#             "is_first": False,
#             "is_last": False,
#             "is_terminal": False,
#         }
#         return obs

#     def compute_reward(self) -> float:
#         reward = 0.0
#         if self.grasped_object:
#             if self.release_same_bin():
#                 reward -= 1.0
#             elif self.release_different_bin():
#                 reward += 10.0
#         else:
#             if self.grasp_object():
#                 reward += 1.0
#         return reward

#     def grasp_object(self) -> bool:
#         # グリッパが開いていない時に物体を掴んだと仮定
#         gripper_pos = self.robot.get_robot_state()["gripper_pos"]
#         if gripper_pos < AL5D.GRIPPER_CLOSE:
#             self.grasped_object = True
#             self.grasped_bin = self.determine_bin()
#             return True
#         return False

#     def release_same_bin(self) -> bool:
#         # 同じ容器内で離した場合
#         if self.grasped_object:
#             self.grasped_object = False
#             return True
#         return False

#     def release_different_bin(self) -> bool:
#         # 異なる容器内で離した場合
#         if self.grasped_object:
#             self.grasped_object = False
#             return True
#         return False

#     def determine_bin(self) -> Side:
#         # エンドエフェクタの位置に基づいてコンテナを判断
#         pos = self.robot.get_robot_state()["end_effector_pos"]
#         if pos["x"] > 100:
#             return Side.RIGHT
#         elif pos["x"] < -100:
#             return Side.LEFT
#         else:
#             return Side.OTHER

#     def render(self):
#         # 画像を表示
#         ret, frame = self.camera.read()
#         if ret:
#             cv2.imshow("RGB Image", frame)
#             cv2.waitKey(1)

#     def close(self):
#         self.robot.close()
#         self.camera.release()
#         cv2.destroyAllWindows()
#         print("環境を終了しました。")


# def main():
#     env = Environment()
#     obs = env.reset()
#     try:
#         while True:
#             env.render()
#             # 行動の例（ランダム）
#             action = {
#                 "action": np.random.choice(
#                     [
#                         "move_x_positive",
#                         "move_x_negative",
#                         "move_y_positive",
#                         "move_y_negative",
#                         "move_z_positive",
#                         "move_z_negative",
#                         "gripper_open",
#                         "gripper_close",
#                     ]
#                 )
#             }
#             obs = env.step(action)
#             print(f"観測: {obs}")
#             if obs["is_terminal"]:
#                 break
#     except KeyboardInterrupt:
#         print("中断されました。")
#     finally:
#         env.close()


if __name__ == "__main__":
    robot = AL5D()
    angles = [
        math.radians(robot.angles[0]),
        math.radians(robot.angles[1]),
        math.radians(robot.angles[2]),
    ]
    print(robot.angles)
    print(robot.forward_kinematics(angles))
    robot.move_coordinate(95, 225, 12)
    angles = [
        math.radians(robot.angles[0]),
        math.radians(robot.angles[1]),
        math.radians(robot.angles[2]),
    ]
    print(robot.angles)
    print(robot.forward_kinematics(angles))
