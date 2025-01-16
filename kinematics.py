import math
import csv
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

LINK1_LENGTH = 145
LINK2_LENGTH = 130
LINK3_LENGTH = 60


JOINT_NUM = 3  # 関節の数

# ジョイントの可動範囲の定義
joint_range = [
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント0
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント1
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント2
    {"min": -math.pi / 2, "max": math.pi / 2},  # ジョイント3
]


def forward_kinematics(theta):
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


def inverse_kinematics(x, y, z):
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

    return theta


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
                        fk_result = forward_kinematics(solution)
                        predicted_positions.append(fk_result)
                        x_fk = round(fk_result[0], 2)
                        y_fk = round(fk_result[1], 2)
                        z_fk = round(fk_result[2], 2)
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
# explore_and_save_solutions()

result = inverse_kinematics(35, -95, 255)
print(result)
print(
    math.degrees(result[0]),
    math.degrees(result[1]),
    math.degrees(result[2]),
)
print(forward_kinematics(result))
