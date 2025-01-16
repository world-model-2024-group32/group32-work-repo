import math

FLOOR_TO_FIRST_JOINT = 2.61 * 0.0254 * 1000
UPPER_ARM_LENGTH = 5.74 * 0.0254 * 1000
FORE_ARM_LENGTH = 7.23 * 0.0254 * 1000
GRIPPER_LENGTH = 4.43 * 0.0254 * 1000

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi


def inverse_kinematics(x, y, z, wrist_angle):
    base_angle = math.atan2(y, x)
    d = math.sqrt(x**2 + y**2)
    wrist_y = z - GRIPPER_LENGTH * math.sin(wrist_angle * DEG_TO_RAD)
    wrist_x = d - GRIPPER_LENGTH * math.cos(wrist_angle * DEG_TO_RAD)
    elbow_angle = (
        wrist_x**2 + wrist_y**2 - UPPER_ARM_LENGTH**2 - FORE_ARM_LENGTH**2
    ) / (2 * UPPER_ARM_LENGTH * FORE_ARM_LENGTH)
    a1 = math.atan2(
        FORE_ARM_LENGTH * math.sin(elbow_angle),
        UPPER_ARM_LENGTH - FORE_ARM_LENGTH * math.cos(elbow_angle),
    )
    a2 = math.atan2(wrist_y, wrist_x)
    shoulder_angle = a1 + a2
    wrist_angle_out = (
        shoulder_angle + elbow_angle + wrist_angle * DEG_TO_RAD + math.pi / 2
    )
    return [base_angle, shoulder_angle, elbow_angle, wrist_angle_out]
