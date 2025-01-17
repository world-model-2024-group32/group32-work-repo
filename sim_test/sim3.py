#オブジェクトの位置を指定して、逆運動学を解いて，アームを動かす
import pybullet as p
import time
import pybullet_data
import numpy as np

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
robot_id = p.loadURDF("../al5d_description/urdf/al5d_robot.urdf", basePosition=[0.6, 0, 0.6], baseOrientation=robot_orientation, useFixedBase=True, globalScaling=2.0)

# 平面追加
plane_id = p.loadURDF("plane.urdf")
table_id = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0])

# オブジェクトを追加
object1_id = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.2, -0.2, 0.6], globalScaling=0.8)
object2_id = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.1, 0.3, 0.6], globalScaling=0.8)

num_joints = p.getNumJoints(robot_id)
end_effector_link_index = num_joints - 1

p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0.1,0,1.5])

def move_to_target(target_position):
    # 逆運動学を解いて関節角度を計算
    joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link_index, target_position)
    
    # 計算された関節角度を適用
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_angles[i])

def reset_to_initial_position():
    # 初期姿勢に戻すための関節角度を設定
    initial_joint_angles = [0] * num_joints
    initial_joint_angles[2] -= 1.5
    
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, initial_joint_angles[i])


while True:
    keys = p.getKeyboardEvents()
    for key, state in keys.items():
        if state & p.KEY_WAS_TRIGGERED or state & p.KEY_IS_DOWN:
            if key == ord('1'):
                object1_pos, _ = p.getBasePositionAndOrientation(object1_id)
                move_to_target(object1_pos)
            elif key == ord('2'):
                object2_pos, _ = p.getBasePositionAndOrientation(object2_id)
                move_to_target(object2_pos)
            elif key == ord('0'):  #初期姿勢に戻る
                reset_to_initial_position()

    p.stepSimulation()
    time.sleep(1./240.)