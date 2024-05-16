import numpy as np

from nao.env.driver.impl.DriveKeypoint import DriveKeypoint
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.RobotEquipment import RobotEquipment
from nao.env.tool.space_tools import get_ground_pose


class DriveKeypointVel(RobotDrive):
    def __init__(self, equipment: RobotEquipment, driveKeypoint: DriveKeypoint):
        super().__init__(equipment)
        self.driveKeypoint = driveKeypoint

        # 记录上一次的位置信息
        self.__last_kp = None  # 关键点相对世界坐标系的位置
        self.__kp_v = None   # 速度相对世界坐标系

    def reset(self):
        # 初始清空上一次及速度记录，读取（速度）时做判空处理
        self.__last_kp = None
        self.__kp_v = None

    @property
    def keypoint_v_world(self):
        """
        :return: 关键点速度字典，相对和表示在世界坐标系下，默认零速度
        """
        if self.__kp_v is None:
            self.__kp_v = {name: np.array([0.0, 0.0, 0.0]) for name in self.nao_keypoint.keypoint_name}

        return self.__kp_v

    @property
    def keypoint_v_ground(self):
        """
        :return: 关键点速度字典，相对世界坐标系，表示在大地投影坐标系下，默认零速度
        """
        v_ground = {}
        # 获取表示在世界的速度向量
        v_world = self.keypoint_v_world
        # 将速度变换到大地坐标系下
        _, R = get_ground_pose()
        for name in v_world:
            v_ground[name] = np.dot(R.T, v_world[name])

        return v_ground

    def update(self, delta):
        """
        :param delta: 两次采样时间间隔，单位毫秒
        """
        # 获取当前与上一次位置，表示在世界坐标系下
        keypoint_world = self.driveKeypoint.get_keypoint_in_world()

        if self.__last_kp is not None:
            now = keypoint_world
            last = self.__last_kp
            # 计算位置随时间变化
            t = delta / 1000  # 秒
            v_world = {}
            for name in now:
                v_world[name] = (now[name] - last[name]) / t

            self.__kp_v = v_world  # 更新速度信息

        # 更新上一次记录
        self.__last_kp = keypoint_world
