import numpy as np

from NAO_RL.Driver.RobotDriveImpl.DrivePose import DrivePose
from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotEquipment import RobotEquipment


class DriveLimbVel(RobotDrive):
    def __init__(self, drivePose: DrivePose, equipment: RobotEquipment):
        super().__init__(equipment)
        self.drivePose = drivePose

        # 记录上一次的关节转角
        self.__last_angles = None
        self.__limb_v = None  # 速度记录

    def reset(self):
        self.__last_angles = None
        self.__limb_v = None

    @property
    def limb_v(self):
        """
        :return: 速度是角度变化率，默认返回零速度
        """
        if self.__limb_v is None:  # 速度未更新
            return [0.0] * self.drivePose.joint_number

        return self.__limb_v
        
    def update(self, delta):
        """
        :param delta: 两次采样的时间差，单位毫秒
        :return: 更新自身的记录
        """
        # 获取当前关节转角
        angles = list(self.drivePose.joint_radians.values())

        if self.__last_angles is not None:  # 判断角度已更新
            # 获取当前与上一次的关节转角
            now = np.array(angles)
            last = np.array(self.__last_angles) if self.__last_angles is not None else now

            # 计算关节转角随时间变化
            t = delta / 1000  # 秒
            v = (now - last) / t
            self.__limb_v = list(v)  # 更新速度信息

        # 更新上一次角度记录
        self.__last_angles = angles
