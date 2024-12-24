import math
import numpy as np
import csv
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

BASE_ANGLE = 0
MAX_ANGLE = 90
MIN_ANGLE = -90
TOTAL_CHANNELS = 6  # チャンネル数
BASE_SPEED = 300
LINK1_LENGTH = 145  # mm
LINK2_LENGTH = 130  # mm
LINK3_LENGTH = 60  # mm
THETA4 = math.radians(0)

angles = [0, 0, 0, 0]


def forward_kinematics(theta1_deg, theta2_deg, theta3_deg, theta4_deg):
    # 角度をラジアンに変換
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    theta3 = math.radians(theta3_deg)
    theta4 = math.radians(theta4_deg)
    print([theta1, theta2, theta3, theta4])
    # theta1 = theta1_deg
    # theta2 = theta2_deg
    # theta3 = theta3_deg
    # theta4 = theta4_deg

    # リンクの長さ
    L1 = LINK1_LENGTH
    L2 = LINK2_LENGTH
    L3 = LINK3_LENGTH

    # ベース位置
    x0 = 0
    y0 = 0
    z0 = 0

    # リンク1の端点位置
    x1 = x0 + L1 * math.cos(theta1) * math.cos(theta2)
    y1 = y0 + L1 * math.sin(theta1) * math.cos(theta2)
    z1 = z0 + L1 * math.sin(theta2)

    # リンク2の端点位置
    x2 = x1 + L2 * math.cos(theta1) * math.cos(theta2 + theta3)
    y2 = y1 + L2 * math.sin(theta1) * math.cos(theta2 + theta3)
    z2 = z1 + L2 * math.sin(theta2 + theta3)

    # リンク3（エンドエフェクタ）の位置
    x3 = x2 + L3 * math.cos(theta1) * math.cos(theta2 + theta3 + theta4)
    y3 = y2 + L3 * math.sin(theta1) * math.cos(theta2 + theta3 + theta4)
    z3 = z2 + L3 * math.sin(theta2 + theta3 + theta4)

    # 結果を返す
    position = {"x": x3, "y": y3, "z": z3}
    return position


def inverse_kinematics(x: float, y: float, z: float):
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

    diff_x = L3 * math.cos(angles[1] + angles[2] + angles[3]) * math.cos(angles[0])
    diff_y = L3 * math.cos(angles[1] + angles[2] + angles[3]) * math.sin(angles[0])
    diff_z = L3 * math.sin(angles[1] + angles[2] + angles[3])
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
        fk_result = forward_kinematics(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
        x_fk, y_fk, z_fk = fk_result["x"], fk_result["y"], fk_result["z"]
        difference = abs(x_fk - x) + abs(y_fk - y) + abs(z_fk - z)
        if difference < min_difference:
            min_difference = difference
            best_solution = solution

    return best_solution


def save_solutions_to_csv(solutions, filename="solutions.csv"):
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            ["input_x", "input_y", "input_z", "output_x", "output_y", "output_z"]
        )
        for solution in solutions:
            writer.writerow(solution)


def explore_and_save_solutions():
    solutions = []
    true_positions = []
    predicted_positions = []

    for x in range(-275, 276, 10):
        for y in range(-275, 276, 10):
            for z in range(-275, 276, 10):
                try:
                    solution = inverse_kinematics(x, y, z)
                    if solution:
                        true_positions.append((x, y, z))
                        theta1_deg, theta2_deg, theta3_deg, theta4_deg = solution[
                            "angles"
                        ]
                        fk_result = forward_kinematics(
                            theta1_deg, theta2_deg, theta3_deg, theta4_deg
                        )
                        predicted_positions.append(
                            (fk_result["x"], fk_result["y"], fk_result["z"])
                        )
                        x_fk = round(fk_result["x"], 2)
                        y_fk = round(fk_result["y"], 2)
                        z_fk = round(fk_result["z"], 2)
                        solutions.append([x, y, z, x_fk, y_fk, z_fk])
                except ValueError:
                    continue

    save_solutions_to_csv(solutions)
    evaluate_error(true_positions, predicted_positions)


def evaluate_error(true_positions, predicted_positions):
    x_true, y_true, z_true = zip(*true_positions)
    x_pred, y_pred, z_pred = zip(*predicted_positions)

    mae_x = mean_absolute_error(x_true, x_pred)
    mae_y = mean_absolute_error(y_true, y_pred)
    mae_z = mean_absolute_error(z_true, z_pred)

    mse_x = mean_squared_error(x_true, x_pred)
    mse_y = mean_squared_error(y_true, y_pred)
    mse_z = mean_squared_error(z_true, z_pred)

    rmse_x = math.sqrt(mse_x)
    rmse_y = math.sqrt(mse_y)
    rmse_z = math.sqrt(mse_z)

    r2_x = r2_score(x_true, x_pred)
    r2_y = r2_score(y_true, y_pred)
    r2_z = r2_score(z_true, z_pred)

    print(f"MAE: x={mae_x:.2f}, y={mae_y:.2f}, z={mae_z:.2f}")
    print(f"MSE: x={mse_x:.2f}, y={mse_y:.2f}, z={mse_z:.2f}")
    print(f"RMSE: x={rmse_x:.2f}, y={rmse_y:.2f}, z={rmse_z:.2f}")
    print(f"R²: x={r2_x:.2f}, y={r2_y:.2f}, z={r2_z:.2f}")


# 全探索を実行してCSVに保存
explore_and_save_solutions()
# テスト座標 5,-155,165
# x_test = 5
# y_test = -155
# z_test = 165
# solutions = inverse_kinematics(x_test, y_test, z_test)
# print(f"逆運動学の解: {solutions}")

# # # 各解について順運動学で検証

# theta1_deg, theta2_deg, theta3_deg, theta4_deg = solutions["angles"]
# fk_result = forward_kinematics(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
# x_fk, y_fk, z_fk = fk_result["x"], fk_result["y"], fk_result["z"]
# print(
#     f"  角度: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°, θ3={theta3_deg:.2f}°, θ4={theta4_deg:.2f}°"
# )
# print(f"  順運動学の結果: x={x_fk:.2f}, y={y_fk:.2f}, z={z_fk:.2f}")
# print(
#     f"  目標座標との差: Δx={x_fk - x_test:.2f}, Δy={y_fk - y_test:.2f}, Δz={z_fk - z_test:.2f}"
# )

# print(
#     forward_kinematics(
#         solutions["angles"][0],
#         solutions["angles"][1],
#         solutions["angles"][2],
#         solutions["angles"][3],
#     )
# )
