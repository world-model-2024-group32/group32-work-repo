import pybullet as p
import time
import pybullet_data
import numpy as np

# PyBulletのGUIモードで接続
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ロボットのURDFファイルをロード
robot_id = p.loadURDF("../al5d_description/urdf/al5d_robot.urdf", basePosition=[0,0,0.1], useFixedBase=True)

# 平面追加（クリック時の衝突点を取得するため）
plane_id = p.loadURDF("plane.urdf")

num_joints = p.getNumJoints(robot_id)
end_effector_link_index = num_joints - 1  # 末端リンクをエンドエフェクタとする

# カメラ位置を調整
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0,0,0.1])

def get_ray_from_screen(x, y, view_matrix, projection_matrix):
    screen_width = 1024
    screen_height = 768
    ndc_x = (2.0 * x) / screen_width - 1.0
    ndc_y = 1.0 - (2.0 * y) / screen_height

    clip_coords = np.array([ndc_x, ndc_y, -1.0, 1.0])
    inv_projection_matrix = np.linalg.inv(np.array(projection_matrix).reshape(4,4))
    inv_view_matrix = np.linalg.inv(np.array(view_matrix).reshape(4,4))
    eye_coords = np.dot(inv_projection_matrix, clip_coords)
    eye_coords = np.array([eye_coords[0], eye_coords[1], -1.0, 0.0])

    world_coords = np.dot(inv_view_matrix, eye_coords)[:3]
    camera_position = p.getDebugVisualizerCamera()[11]
    ray_from = np.array(camera_position)
    ray_to = ray_from + world_coords
    return ray_from, ray_to

while True:
    mouse_events = p.getMouseEvents()
    for event in mouse_events:
        if event[0] & 0x01:
            print("clicked")
            x, y = event[1], event[2]

            # 現在のカメラ情報取得
            cam_info = p.getDebugVisualizerCamera()
            view_matrix = cam_info[2]
            projection_matrix = cam_info[3]

            # クリック位置からレイ生成
            ray_from, ray_to = get_ray_from_screen(x, y, view_matrix, projection_matrix)
            
            # レイキャストして平面との交点を取得
            # p.rayTestは ray_from から ray_to までの直線上で衝突検出を行う
            # ray_toを遠くに延ばすことで地面との交点を確実に探す
            extended_ray_to = ray_from + (ray_to - ray_from) * 10.0
            hits = p.rayTest(ray_from, extended_ray_to)
            
            # hitsは[(hitObjectUniqueId, hitLinkIndex, hitFraction, hitPosition, hitNormal), ...]を返す
            # 地面と衝突した場合、hitPositionで衝突点が得られる
            if hits and hits[0][0] != -1:
                hit_pos = hits[0][3]
                target_position = hit_pos

                # IK計算
                # 関節角はp.calculateInverseKinematicsで直接求められる
                joint_angles = p.calculateInverseKinematics(
                    robot_id,
                    end_effector_link_index,
                    target_position,
                    # 姿勢指定しない場合はデフォルト姿勢
                )

                # 計算された関節角度を適用
                for i in range(num_joints):
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_angles[i])

    p.stepSimulation()
    time.sleep(1./240.)
