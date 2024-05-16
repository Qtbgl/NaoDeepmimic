import numpy as np

from NAO_RL.Driver.RobotDriveImpl.DrivePose import DrivePose
from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotEquipment import RobotEquipment
from NAO_RL.Tools.space_tools import get_torso_pose_in_ground
from controller import Supervisor


class DriveKeypoint(RobotDrive):
    def __init__(self, equipment: RobotEquipment, supervisor: Supervisor, drivePose: DrivePose):
        super().__init__(equipment)
        self.supervisor = supervisor
        self.drivePose = drivePose

    def get_keypoint_in_world(self):
        """
        :return: 字典返回关键点位置，相对世界坐标系，可用于绝对速度计算
        """
        # 获取Torso在世界坐标系下位姿
        torso_T_world = np.array(self.supervisor.getFromDef("Torso").getPose()).reshape((4, 4))
        torso_R_world = torso_T_world[:3, :3]
        torso_t_world = torso_T_world[:3, 3]

        # 获取关键点相对时间的位置
        keypoint_world = self.convert_keypoint_from_torso(torso_t_world, torso_R_world)

        return keypoint_world

    def get_keypoint_in_ground(self):
        """
        :return: 字典返回由FK-NaoKeypoint中定义的15个关键点，相对大地投影坐标系
        """
        # 获取躯干相对大地的姿态
        torso_t_ground, torso_R_ground = get_torso_pose_in_ground()

        # 获取关键点相对大地的位置
        keypoint_ground = self.convert_keypoint_from_torso(torso_t_ground, torso_R_ground)

        return keypoint_ground

    def convert_keypoint_from_torso(self, torso_t_frame, torso_R_frame) -> dict[str, np.ndarray]:
        """
        :param torso_t_frame: 躯干的位置
        :param torso_R_frame: 躯干的姿态
        :return: 关键点在特定坐标系下的位置向量，字典返回
        """
        # 获取关键点相对躯干的位置
        joint_angles = self.drivePose.joint_radians
        keypoint = self.nao_keypoint.getKeypoint(joint_angles)

        # 计算关键点在特定坐标系的下位置
        position = {}
        for name, t_torso in keypoint.items():
            # 将相对躯干的位置映射到特定坐标系下
            t_frame = np.dot(torso_R_frame, t_torso)
            # 躯干自身的位移加相对躯干的位置
            t_frame = torso_t_frame + t_frame
            position[name] = t_frame

        return position
