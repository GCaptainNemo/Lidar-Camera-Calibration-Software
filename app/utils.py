import numpy as np
import cv2


def rvec2mat(r_vec: np.array):
    """
    Roderigus Formula: Lee algebra -> Lee group
    :param r_vec: angle axis/rotate vector
    :return:
    """
    theta = np.linalg.norm(r_vec)
    r_wedge = np.array([[0.0, -r_vec[2, 0], r_vec[1, 0]],
                        [r_vec[2, 0], 0.0, -r_vec[0, 0]],
                        [-r_vec[1, 0], r_vec[0, 0], 0.0]], dtype=np.float32)  # asymmetric matrix
    rotate_mat = np.eye(3, dtype=np.float32) + \
                 r_wedge / theta * np.sin(theta) + \
                 r_wedge @ r_wedge * (1 - np.cos(theta)) / theta ** 2
    return rotate_mat


def mat2rvec(mat):
    """
    Roderigus Formula
    :param mat:
    :return:
    """
    r_vec, j = cv2.Rodrigues(mat)
    return r_vec


def euler2mat(euler: np.array):
    """

    euler angle(RPY format unit:degree) -> rotate matrix
    :param euler: euler angle(roll, pitch, yaw)
    :return: rotate matrix = yaw_mat @ pitch_mat @ roll_mat
    """
    rad_euler = euler.reshape(-1) / 180 * np.pi
    roll = rad_euler[0]  # X
    pitch = rad_euler[1]  # Y
    yaw = rad_euler[2]  # Z

    roll_mat = np.array([[1, 0, 0],
                         [0, np.cos(roll), -np.sin(roll)],
                         [0, np.sin(roll), np.cos(roll)],
                         ])
    pitch_mat = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                          [0, 1, 0],
                          [-np.sin(pitch), 0, np.cos(roll)],
                          ])
    yaw_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1],
                        ])
    rotate_mat = yaw_mat @ pitch_mat @ roll_mat
    return rotate_mat


def mat2euler(R: np.array, is_normalize=True):
    """
    rotate matrix -> euler angle(RPY format unit:degree)
    :param rotate_mat: yaw_mat @ pitch_mat @ roll_mat
    :return: euler angle(roll, pitch, yaw)
    reference: https://zhuanlan.zhihu.com/p/259999988
    """
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    if is_normalize:
        x = norm_0_2pi(x)
        y = norm_0_2pi(y)
        z = norm_0_2pi(z)
    return np.array([x, y, z]) * 180 / np.pi

def norm_0_2pi(angle: float) -> float:
    if angle < 0:
        angle += np.pi * 2
    elif angle > 2 * np.pi:
        angle -= np.pi * 2
    return angle


if __name__ == "__main__":
    # rpy = np.array([270, 90, 180]).reshape(3, 1)
    rpy = np.array([90.188373, 0.5153747, 179.1440369]).reshape(3, 1)
    # mat = np.array([[0.0, -1.0, 0.0],
    #                             [0.0, 0.0, -1.0],
    #                             [1.0, 0.0, 0.0]], dtype=np.float32)
    # mat = np.array([0.01285381, -0.9999103, -0.00375912,
    # -0.00171844, 0.00373733, -0.99999154,
    # 0.9999159, 0.01286116, -0.00167]).reshape(3, 3)
    mat = euler2mat(rpy)
    vec = mat2euler(mat)
    print(mat, vec)
    print(mat.shape, vec.shape)
    rvec = mat2rvec(mat)
    print(rvec)
