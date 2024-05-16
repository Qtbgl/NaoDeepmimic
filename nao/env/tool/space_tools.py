"""
游戏空间计算的工具类，基础功能模块，广泛调用
"""
import numpy as np
from scipy.spatial.transform import Rotation

from controller import Node, Supervisor
from controller.device import Device


_supervisor: Supervisor


def fit(supervisor: Supervisor):
    """
    在调用工具方法前注入其依赖
    """
    global _supervisor
    _supervisor = supervisor


def get_ground_pose():
    """
    注: 使用Supervisor直接获取
    :return: 大地坐标系（躯干水平旋转与地面投影）相对世界坐标系的位置与姿态
    """
    # 获取Torso在世界坐标系下的位置，与水平面上的旋转
    torso_T_world = np.array(_supervisor.getFromDef("Torso").getPose()).reshape((4, 4))
    torso_R_world = torso_T_world[:3, :3]
    torso_t_world = torso_T_world[:3, 3]
    # 分解为欧拉角x-y-z轴外在旋转
    rpy = Rotation.from_matrix(torso_R_world).as_euler('xyz', degrees=False)
    # 世界坐标系在水平面上的旋转，即旋转z轴上的yaw弧度
    ground_R_world = Rotation.from_euler('z', rpy[2]).as_matrix()
    ground_t_world = torso_t_world
    ground_t_world[2] = 0
    return ground_t_world, ground_R_world


def get_torso_pose_in_ground():
    """
    注: 使用Supervisor直接获取
    :return: Torso相对投影大地坐标系（躯干水平旋转与地面投影）的位置与姿态
    """
    # 获取Torso在世界坐标系下的位置，与水平面上的旋转
    torso_T_world = np.array(_supervisor.getFromDef("Torso").getPose()).reshape((4, 4))
    torso_R_world = torso_T_world[:3, :3]
    torso_t_world = torso_T_world[:3, 3]
    # 分解为欧拉角x-y-z轴外在旋转
    rpy = Rotation.from_matrix(torso_R_world).as_euler('xyz', degrees=False)
    # 大地坐标系只在z轴旋转，躯干相对大地坐标系再在Y-X轴内禀旋转，即x-y轴对应roll-pitch的外在旋转
    torso_R_ground = Rotation.from_euler('xy', rpy[:2]).as_matrix()  # debugged
    # 躯干相对大地只有z轴上的位移
    torso_t_ground = np.zeros(3, dtype=float)
    torso_t_ground[2] = torso_t_world[2]
    return torso_t_ground, torso_R_ground


def get_node_pose_in_world(node: Node):
    """
    :param node: 结点需继承自Webots-Pose
    :return: 结点相对世界的位置和姿态
    """
    T = np.array(node.getPose()).reshape((4, 4))
    R = T[:3, :3]
    t = T[:3, 3]
    # 返回设备的位置和旋转
    position = list(t)
    rotation = Rotation.from_matrix(R)
    return position, rotation


def get_solid_v_in_ground(solid_def: str):
    """
    :return: 刚体的速度与角速度，相对于世界坐标系，表达在大地坐标系下
    """
    # 获取刚体在世界坐标系下的速度
    velocity = _supervisor.getFromDef(solid_def).getVelocity()
    v_world = np.array(velocity[:3])
    a_world = np.array(velocity[3:])

    # 获取大地相对世界的姿态
    _, R = get_ground_pose()

    # 速度向量映射到ground坐标系下
    v_ground = np.dot(R.T, v_world)
    a_ground = np.dot(R.T, a_world)

    return v_ground, a_ground


def get_vector_in_world(v: np.ndarray, device: Device):
    """
    :param v: 向量相对表示
    :param device: 相对的设备
    """
    node = _supervisor.getFromDevice(device._tag)
    T = np.array(node.getPose()).reshape((4, 4))
    R = T[:3, :3]
    # 将v映射到世界坐标系下
    v = np.dot(R, v)
    return v


def get_vector_in_local(v: np.ndarray, device: Device):
    """
    :param v: 向量表示在世界坐标系
    :param device: 要相对的设备
    """
    node = _supervisor.getFromDevice(device._tag)
    T = np.array(node.getPose()).reshape((4, 4))
    R = T[:3, :3]
    # 将v映射到设备坐标系下
    v = np.dot(R.T, v)
    return v


def apart_touch_force(v: np.ndarray, device: Device):
    """
    :param v: 力向量表示在设备坐标系
    :param device:
    :return: 压力和摩檫力，向量相对表示
    """
    # 表示在世界坐标系下
    v_world = get_vector_in_world(v, device)
    # 分解水平和垂直的力
    pressure = np.zeros(3)
    friction = np.zeros(3)
    # 取z轴分量
    pressure[2] = v_world[2]
    # 取x,y分量
    friction[:2] = v_world[:2]
    # 表示回相对坐标系
    pressure = get_vector_in_local(pressure, device)
    friction = get_vector_in_local(friction, device)
    return pressure, friction


def get_effector_position(effector_names):
    position = {}  # 世界位置
    for name in effector_names:
        T = np.array(_supervisor.getFromDef(name).getPose()).reshape((4, 4))
        t = T[:3, 3]
        position[name] = t
    return position


def get_mass_center():
    nao_node = _supervisor.getSelf()
    position = np.array(nao_node.getCenterOfMass())
    return position
