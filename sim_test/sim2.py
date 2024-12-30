import pybullet as p
import time
import pybullet_data
import numpy as np
from scipy.optimize import minimize

# PyBulletのGUIモードで接続
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ロボットのURDFファイルをロード (AL5D)
urdf_file = "../al5d_description/urdf/al5d_robot.urdf"
robot_id = p.loadURDF(urdf_file, basePosition=[0, 0, 0.1], useFixedBase=True)

num_joints = p.getNumJoints(robot_id)

# リンク長 (mm) - 実際にはURDF等から正確に取得すべき
L1 = 145.0
L2 = 130.0
L3 = 60.0
L4 = 100.0
joint_lengths = [L1, L2, L3, L4]

# カメラ調整（近接させる）
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0,0,0.1])

def rotation_matrix(axis, angle):
    """
    軸(axis)周りにangle[rad]回転する回転行列を返す
    axisは'x', 'y', 'z'のいずれか
    """
    c = np.cos(angle)
    s = np.sin(angle)
    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, c, -s],
                         [0, s, c]])
    elif axis == 'y':
        return np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s,0, c]])
    elif axis == 'z':
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])
    else:
        return np.eye(3)

def forward_kinematics(angles, joint_lengths):
    """
    非常に簡略化したFK関数。  
    仮定:  
    J1: z軸回り回転  
    J2: y軸回り回転  
    J3: y軸回り回転  
    J4: y軸回り回転  

    各ジョイント回転後、x方向にリンクが伸びると仮定する。
    """
    # anglesはdegree
    th1, th2, th3, th4 = np.radians(angles)
    
    # 初期姿勢
    pos = np.array([0,0,0])
    R = np.eye(3)

    # J1 (Z軸回り)
    R = R @ rotation_matrix('z', th1)
    pos = pos + R @ np.array([0,0,0])  # ベースから始まるのでオフセットなし
    # L1リンクはx方向に伸びると仮定
    pos = pos + R @ np.array([joint_lengths[0],0,0])

    # J2 (Y軸回り)
    R = R @ rotation_matrix('y', th2)
    pos = pos + R @ np.array([joint_lengths[1],0,0])

    # J3 (Y軸回り)
    R = R @ rotation_matrix('y', th3)
    pos = pos + R @ np.array([joint_lengths[2],0,0])

    # J4 (Y軸回り)
    R = R @ rotation_matrix('y', th4)
    pos = pos + R @ np.array([joint_lengths[3],0,0])

    return pos

def objective(angles, target_position, joint_lengths):
    # FK計算
    end_effector_pos = forward_kinematics(angles, joint_lengths)
    return np.sum((end_effector_pos - target_position)**2)

def get_ray_from_screen(x, y, view_matrix, projection_matrix):
    # スクリーン座標を正規化デバイス座標に変換
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

initial_guess = [0,0,0,0]
bounds = [(-150,150)]*num_joints

while True:
    mouse_events = p.getMouseEvents()
    for event in mouse_events:
        if event[0] & 0x01:
            print("clicked")
            x, y = event[1], event[2]
            view_matrix = p.computeViewMatrixFromYawPitchRoll([0,0,0],1.0,90,-30,0,2)
            projection_matrix = p.computeProjectionMatrixFOV(60,1,0.1,100)
            ray_from, ray_to = get_ray_from_screen(x, y, view_matrix, projection_matrix)

            # ray_toはカメラ方向に伸びた点であり、そのままでは地面などに当たらないことが多い
            # 必要に応じてレイキャストして、地面やオブジェクトとの交点を取得してもよい。
            # ここでは簡易的にray_to自体を目標位置とする。
            target_position = ray_to

            result = minimize(objective, initial_guess, args=(target_position, joint_lengths), bounds=bounds)
            optimal_angles = result.x
            initial_guess = optimal_angles

            for i in range(num_joints):
                p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, np.radians(optimal_angles[i]))

    p.stepSimulation()
    time.sleep(1./240.)
