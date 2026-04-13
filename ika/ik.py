import numpy as np
import math

def get_joints_to(x: float, y: float, z: float) \
        -> tuple[int, int, int, int, int]:
    target = np.array([x, y, z]).T
    # targeting down-facing position
    base_joint = np.array([55, -45, 110]).T
    l1_length = 110
    l2_length = 105
    j3_rad = math.radians(65)  # math.pi / 4
    j4_rad = math.radians(85)
    j5_off = np.array([27, 12, -57]).T
    turret_angle = math.atan2(y - base_joint[1], x - base_joint[0]) \
        - math.asin(j5_off[1] / np.linalg.norm(target - base_joint))
    j1 = int(math.degrees(turret_angle))
    end_joint = np.array([x, y, z]).T - \
        np.array([[math.cos(turret_angle), -math.sin(turret_angle), 0],
                  [math.sin(turret_angle), math.cos(turret_angle), 0],
                  [0, 0, 1]]) @ j5_off
    vec_to_end_joint = end_joint - base_joint
    dist_to_end_joint = np.linalg.norm(vec_to_end_joint)
    if dist_to_end_joint > l1_length + l2_length:
        raise ValueError("Impossible to generate solution")
    flat_delta = np.array([[math.cos(turret_angle), math.sin(turret_angle), 0],
                           [-math.sin(turret_angle), math.cos(turret_angle), 0],
                           [0, 0, 1]]) @ vec_to_end_joint

    theta2 = math.pi - math.acos(
        (dist_to_end_joint ** 2 - l1_length ** 2 - l2_length ** 2) /
        (-2 * l1_length * l2_length))
    theta1 = math.atan2(flat_delta[2], flat_delta[0]) + math.asin(
        math.sin(math.pi - theta2) * l2_length / dist_to_end_joint)
    # print(math.degrees(theta2))
    # print(math.degrees(theta1))
    theta3 = 3 * math.pi/2 - j4_rad + theta1 - theta2
    # print(math.degrees(theta3))
    j2 = int(math.degrees(theta1))
    j3 = int(math.degrees(j3_rad + theta2))
    j4 = int(math.degrees(theta3))
    return j1, j2, j3, j4, 0


if __name__ == "__main__":
    print(get_joints_to(0, 0, 110))
