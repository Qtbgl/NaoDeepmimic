"""
纯数学计算的工具类，基础计算模块，底层调用
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def normalize_to_range(value, min_val, max_val, new_min, new_max, clip=False):
    """
    Normalize value to a specified new range by supplying the current range.

    :param value: value to be normalized
    :param min_val: value's min value, value ∈ [min_val, max_val]
    :param max_val: value's max value, value ∈ [min_val, max_val]
    :param new_min: normalized range min value
    :param new_max: normalized range max value
    :param clip: whether to clip normalized value to new range or not
    :return: normalized value ∈ [new_min, new_max]
    """
    value = float(value)
    min_val = float(min_val)
    max_val = float(max_val)
    new_min = float(new_min)
    new_max = float(new_max)

    if clip:
        value = np.clip((new_max - new_min) / (max_val - min_val) * (value - max_val) + new_max, new_min, new_max)
    else:
        value = (new_max - new_min) / (max_val - min_val) * (value - max_val) + new_max

    return float(value)


def to_axis_angle(rotation: Rotation):
    vec = rotation.as_rotvec()
    angle_rad = np.linalg.norm(vec)
    if angle_rad == 0:
        return [0, 0, 1, 0]

    return list(vec / angle_rad) + [angle_rad]


def to_spherical_(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)
    phi = math.acos(z / r)
    theta = math.atan2(y, x)
    return r, theta, phi


def to_spherical(x, y, z):
    """
    :return: φ的取值范围为0到π, θ取值范围为-π到π
    """
    r = np.sqrt(x**2 + y**2 + z**2)
    phi = np.arccos(z / r)
    theta = np.arctan2(y, x)
    return r, theta, phi


def getAnklePitchRoll(v):
    """
    :param v: 设足部初始相对朝向用向量v表示，它相对于胫骨的坐标系，初始时与z轴同向；
    :return: 计算踝关节依次的转角，使得初始的v转为现在的v
    """
    x, y, z = v
    # 踝关节z轴向量先绕y轴转再绕x轴转，对应球坐标中x轴向量先绕z轴转再绕y轴转
    r, theta, phi = to_spherical(z, x, y)
    # 先转pitch再转roll，对应先转theta再转phi-pi/2
    return theta, phi - math.pi/2


# noinspection DuplicatedCode
def getFeetPressure(fsv):
    """
    :param fsv: 左右脚 force sensor values
    :return: 左右脚四个方位的受力，近似计算
    """
    left = []
    right = []

    # The coefficients were calibrated against the real
    # robot so as to obtain realistic sensor values.
    left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
    left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
    left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
    left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

    right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
    right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
    right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
    right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

    for i in range(0, len(left)):
        left[i] = max(min(left[i], 25), 0)
        right[i] = max(min(right[i], 25), 0)

    return left, right


def get_mse_loss(y_pred, y_true):
    # 列表转为array
    y_pred = np.array(y_pred)
    y_true = np.array(y_true)
    # 均方误差（MSE）
    loss = np.mean((y_true - y_pred) ** 2)
    return loss


def log_decay(x, initial_value, decay_rate):
    """
    :param x: np一维数组，取值>=0
    :param initial_value: x=0时y的取值
    :param decay_rate: 衰减率
    :return: 从initial_value一直衰减到0
    """
    return initial_value * np.exp(-decay_rate * x)
