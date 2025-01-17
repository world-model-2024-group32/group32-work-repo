#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
import time
import pybullet as p
import pybullet_data
import numpy as np

from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

def connect_and_load_robot():
    """
    PyBulletにDIRECTモードで接続し、AL5Dロボットを読み込む。
    戻り値: (robot_id, end_effector_link_index)
    """
    # ウィンドウを開かないモードで接続
    p.connect(p.DIRECT)  
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    robot_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
    robot_id = p.loadURDF(
        "../al5d_description/urdf/al5d_robot.urdf",
        basePosition=[0.6, 0, 0.6],
        baseOrientation=robot_orientation,
        useFixedBase=True,
        globalScaling=2.0
    )

    p.loadURDF("plane.urdf")
    p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0])

    num_joints = p.getNumJoints(robot_id)
    end_effector_link_index = num_joints - 1

    return robot_id, end_effector_link_index


def set_joint_positions(robot_id, joint_positions, steps=120):
    """
    PyBulletでロボットの各ジョイントに joint_positions を適用し、
    指定ステップだけ stepSimulation() する。
    """
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            targetPosition=joint_positions[i]
        )
    
    for _ in range(steps):
        p.stepSimulation()


def move_and_measure(robot_id, end_eff_idx, target_pos):
    """
    target_pos (x,y,z) を与えて IK を計算 → 関節角度をセット → 
    シミュレーション後に実際のエンドエフェクタ位置を返す。
    """
    # 逆運動学 (関節角度のリスト)
    joint_positions = p.calculateInverseKinematics(
        robot_id, 
        end_eff_idx, 
        target_pos
    )
    # 関節角度を適用
    set_joint_positions(robot_id, joint_positions, steps=120)

    # 末端リンクの実際の位置を取得
    link_state = p.getLinkState(robot_id, end_eff_idx)
    reached_pos = link_state[0]  # (x, y, z)
    return reached_pos


def test_ik_and_collect_data(robot_id, end_eff_idx):
    """
    いくつかの (x, y, z) について IK → 実際に到達した位置を計測し、
    (真の目標位置, 実際の到達位置) をリストとして返す。
    """
    solutions = []
    true_positions = []
    pred_positions = []

    # 探索範囲は，sm2.pyを参考にして設定(pick and placeを想定した範囲)
    x_list = np.linspace(0.0, 0.4, 10)  # 10点 (0.0~0.4)
    y_list = np.linspace(-0.4, 0.4, 10)  # 10点 (-0.4~0.4)
    z_list = np.linspace(0.6, 1.0, 5)  # 5点 (0.6~1.0)

    for x in x_list:
        for y in y_list:
            for z in z_list:
                target_pos = (x, y, z)
                reached_pos = move_and_measure(robot_id, end_eff_idx, target_pos)

                tx, ty, tz = target_pos
                rx, ry, rz = reached_pos

                solutions.append([tx, ty, tz, rx, ry, rz])
                true_positions.append((tx, ty, tz))
                pred_positions.append((rx, ry, rz))

    return solutions, true_positions, pred_positions


def save_solutions_to_csv(solutions, filename="solutions_direct.csv"):
    """
    solutions: [[tx, ty, tz, rx, ry, rz], ...] をCSVに書き出す。
    """
    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["input_x", "input_y", "input_z", 
                         "output_x", "output_y", "output_z"])
        writer.writerows(solutions)


def evaluate_error(true_positions, pred_positions):
    """
    (x, y, z) の MAE / MSE / RMSE / R² をまとめて表示。
    """
    from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

    x_true, y_true, z_true = zip(*true_positions)
    x_pred, y_pred, z_pred = zip(*pred_positions)

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

    print("=== Evaluation Results ===")
    print(f"MAE : x={mae_x:.4f}, y={mae_y:.4f}, z={mae_z:.4f}")
    print(f"MSE : x={mse_x:.4f}, y={mse_y:.4f}, z={mse_z:.4f}")
    print(f"RMSE: x={rmse_x:.4f}, y={rmse_y:.4f}, z={rmse_z:.4f}")
    print(f"R²  : x={r2_x:.4f}, y={r2_y:.4f}, z={r2_z:.4f}")


def main():
    # 1) DIRECTモードでPyBulletに接続
    robot_id, end_eff_idx = connect_and_load_robot()

    # 2) IKを試して (真の座標, 到達座標) を収集
    solutions, true_positions, pred_positions = test_ik_and_collect_data(robot_id, end_eff_idx)

    # 3) CSV保存
    save_solutions_to_csv(solutions, filename="solutions_direct.csv")

    # 4) 評価指標を表示
    evaluate_error(true_positions, pred_positions)

    p.disconnect()


if __name__ == "__main__":
    main()
