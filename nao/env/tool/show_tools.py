"""
打印或在游戏中展示，可视化模块，辅助调用
"""
import numpy as np
from scipy.spatial.transform import Rotation

from nao.env.tool.Access.ShowToolAccess import ShowToolAccess
from nao.env.tool.space_tools import get_node_pose_in_world, apart_touch_force, get_ground_pose

from controller import Supervisor, Node
from controller.device import Device


_Access: ShowToolAccess


def fit(supervisor: Supervisor):
    """
    注入到模块自身，某些工具函数会依赖
    """
    global _Access
    _Access = ShowToolAccess(supervisor)


def show_feet(left, right):
    newtonsLeft = sum(left)
    newtonsRight = sum(right)
    print('----------foot sensors----------')
    print('+ left ---- right +')
    print('+-------+ +-------+')
    print('|' + str(round(left[0], 1)) +
          '  ' + str(round(left[1], 1)) +
          '| |' + str(round(right[0], 1)) +
          '  ' + str(round(right[1], 1)) +
          '|  front')
    print('| ----- | | ----- |')
    print('|' + str(round(left[3], 1)) +
          '  ' + str(round(left[2], 1)) +
          '| |' + str(round(right[3], 1)) +
          '  ' + str(round(right[2], 1)) +
          '|  back')
    print('+-------+ +-------+')
    print('total: %f Newtons, %f kilograms'
          % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81)))


def show_leg_rpy_in_world(env: Supervisor):
    # 用于检查脚部是否真的水平
    def get_in_world_pose_rpy(supervisor: Supervisor, DEF: str):
        T = np.array(supervisor.getFromDef(DEF).getPose()).reshape((4, 4))
        R = T[:3, :3]
        rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
        return rpy

    rpy1 = get_in_world_pose_rpy(env, "LLeg")
    rpy2 = get_in_world_pose_rpy(env, "RLeg")
    print('..........................................')
    print('LLeg world', [round(rad, 2) for rad in rpy1])
    print('RLeg world', [round(rad, 2) for rad in rpy2])


def show_rotation_from_torso_in_world(rotation: Rotation):
    """
    :param rotation: 相对Torso的旋转，呈现相对世界的旋转
    """
    T = np.array(_Access.supervisor.getFromDef('Torso').getPose()).reshape((4, 4))
    torso_R = T[:3, :3]
    R = rotation.as_matrix()
    R = np.dot(torso_R, R)
    # 3D呈现
    _Access.cube.update(Rotation.from_matrix(R))


def show_nao_rotation_in_world(nao_R: np.ndarray, means: str):
    """
    :param nao_R: 躯干相对大地/平面的旋转；躯干相对世界的旋转
    :param means: nao_R含义@显示含义
    """
    if means == 'torso_rotation_from_ground@show_ground_in_world':
        T = np.array(_Access.supervisor.getFromDef('Torso').getPose()).reshape((4, 4))
        R1 = T[:3, :3]
        # 计算plane相对world坐标系的姿态
        R0 = nao_R
        rotation = np.dot(R1, R0.T)
        rotation = Rotation.from_matrix(rotation)

    elif means == 'torso_rotation_from_ground@show_torso_in_ground':
        rotation = Rotation.from_matrix(nao_R)

    elif means == 'torso_rotation_from_world@show_torso_in_world':
        rotation = Rotation.from_matrix(nao_R)
    else:
        raise Exception
    # 3D呈现
    _Access.cube.update(rotation)


def show_device_rotation_in_world(device: Device):
    # noinspection PyProtectedMember
    node = _Access.supervisor.getFromDevice(device._tag)  # webots问题，必须用int型的tag
    _, rotation = get_node_pose_in_world(node)
    # 3D呈现设备姿态
    _Access.cube.update(rotation)


def show_vector_of_local_in_world(v: list, node: Node):
    """
    :param v: 表示在某一相对坐标系下的向量
    :param node: 向量v所相对的结点，需带有位姿属性
    """
    # 获取结点的位姿
    position, rotation = get_node_pose_in_world(node)
    # 将向量v定位在相同位姿的坐标系上
    _Access.line.update(v, position, rotation)


def show_force_in_world(force: list, node: Node, unit: float = 1):
    """
    :param force: 力向量
    :param node: 相对坐标系
    :param unit: 一牛等于几米 N * m
    """
    # 获取结点的位姿
    position, rotation = get_node_pose_in_world(node)
    # 将力向量定位在相对坐标系下
    v = np.array(force) * unit
    _Access.line.update(list(v), position, rotation)


def show_feet_contact_force(fsv: list, fsr: list[Device], foot_index=None, show_friction=False):
    for i, v in enumerate(fsv):
        fsv[i] = [v[0], v[1], v[2]]  # 变为列表

    nodes = []
    forces = []
    for i, v in enumerate(fsv):
        node = _Access.supervisor.getFromDevice(fsr[i]._tag)
        pressure, friction = apart_touch_force(np.array(v), fsr[i])
        # 选择显示压力
        force = list(friction) if show_friction else list(pressure)
        nodes.append(node)
        forces.append(force)

    # 选择显示一只脚
    if foot_index is None:
        foot_index = np.argmax([np.linalg.norm(force) for force in forces])  # 挑取最大的力
    # 3D呈现
    show_force_in_world(forces[foot_index], nodes[foot_index], unit=0.02)


def show_keypoint_neo_line(position: dict, frame: str):
    """
    荧光线展示身体关键点
    :param position: 字典以NaoKeypoint中的名字索引
    :param frame: 关键点相对坐标系
    """
    if frame == 'Ground':
        t, R = get_ground_pose()  # 大地相对世界位姿
        # 设置大地坐标系的位姿
        t = list(t)
        R = Rotation.from_matrix(R)
    elif frame == 'Torso':
        t, R = get_node_pose_in_world(_Access.supervisor.getFromDef("Torso"))
    else:
        raise Exception

    _Access.line.update_pose(t, R)
    _Access.cube.update(R)
    # 设置目标点位置
    begin = [
        "LShoulderPitch", "LElbowYaw", "LHand",
        "LHipYawPitch", "LKneePitch", "LFoot", "LToe",
    ]
    end = [
        "RShoulderPitch", "RElbowYaw", "RHand",
        "RHipYawPitch", "RKneePitch", "RFoot", "RToe",
    ]
    p1 = [position[name] for name in begin]  # 大地上的位置
    p2 = [position[name] for name in end]  # 大地上的位置
    _Access.line.update_point_pairs(p1, p2)
