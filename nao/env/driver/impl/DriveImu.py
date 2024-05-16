import math

import numpy as np
from scipy.spatial.transform import Rotation

from NAO_RL.Driver.RobotDrive import RobotDrive


class DriveImu(RobotDrive):
    def getTorsoImuPose(self):
        """
        :return: 躯干相对于大地坐标系的姿态
        """
        # Webots惯性单元
        rpy = self.imu.getRollPitchYaw()
        # print('rpy', rpy)
        # 应对pitch90度万向锁
        if abs(math.pi / 2 - abs(rpy[1])) <= 0.2:  # 当角度差接近11°时
            # print('locking')
            rpy[0] = rpy[2] = 0  # x,z轴锁住相等
            rpy[1] = (85 / 180 * math.pi) * np.sign(rpy[1])  # pitch不能再为90°

        # 欧拉角x-y两轴外在旋转，对应的roll,pitch单位为弧度
        rotation = Rotation.from_euler('xy', rpy[:2], degrees=False)
        return rotation.as_matrix()

    @property
    def torsoRotation(self):
        """
        :return: 躯干相对于大地的姿态，不应对pitch90度万向锁问题
        """
        # Webots惯性单元
        rpy = self.imu.getRollPitchYaw()
        rotation = Rotation.from_euler('xy', rpy[:2], degrees=False)
        return rotation

    @property
    def torsoAngularV(self):
        """
        :return: 躯干角速度表达在躯干坐标系，rad/s
        """
        # gyro不需调整
        a = self.gyro.getValues()
        a = [a[i] for i in range(3)]  # 转换为列表
        return a
