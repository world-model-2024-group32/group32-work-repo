# 矢印キーでアームを操作するプログラム
import pybullet as p
import time
import pybullet_data
import numpy as np

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


robot_orientation = p.getQuaternionFromEuler([0, 0, np.pi]) 
robot_id = p.loadURDF("../al5d_description/urdf/al5d_robot.urdf", basePosition=[0.6, 0, 0.6], baseOrientation=robot_orientation, useFixedBase=True, globalScaling=2.0)


plane_id = p.loadURDF("plane.urdf")
table_id = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0])

object_id = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.0, 0.2, 0.6], globalScaling=0.9)

num_joints = p.getNumJoints(robot_id)
end_effector_link_index = num_joints - 1

# カメラ位置を調整
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-45, cameraTargetPosition=[0,0,1.2])

joint_angles = [0] * num_joints
joint_angles[2] -= 1.5

def update_joint_angles(key):
    global joint_angles
    if key == p.B3G_LEFT_ARROW:
        joint_angles[0] -= 0.1
    elif key == p.B3G_RIGHT_ARROW:
        joint_angles[0] += 0.1
    elif key == p.B3G_UP_ARROW:
        joint_angles[1] += 0.1
        joint_angles[2] -= 0.1
        joint_angles[3] += 0.1 
    elif key == p.B3G_DOWN_ARROW:
        joint_angles[1] -= 0.1 
        joint_angles[2] += 0.1 
        joint_angles[3] -= 0.1
    print(joint_angles)

# urdfにエンドエフェクタのリンクが定義されていないため、動作しない
def grasp_object():
    end_effector_state = p.getLinkState(robot_id, end_effector_link_index)
    end_effector_pos = end_effector_state[0]

    object_pos, object_orn = p.getBasePositionAndOrientation(object_id)

    distance = np.linalg.norm(np.array(end_effector_pos) - np.array(object_pos))

    # 距離が一定以下ならオブジェクトを掴む
    if distance < 0.1:
        p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=end_effector_link_index,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        print("Object grasped!")

while True:
    keys = p.getKeyboardEvents()
    for key, state in keys.items():
        if state & p.KEY_WAS_TRIGGERED or state & p.KEY_IS_DOWN:
            update_joint_angles(key)
            if key == p.B3G_RETURN:
                grasp_object()

    # 計算された関節角度を適用
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_angles[i])

    p.stepSimulation()
    time.sleep(1./240.)