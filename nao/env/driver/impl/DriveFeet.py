from NAO_RL.Driver.RobotDriveImpl.DrivePose import DrivePose
from NAO_RL.Driver.RobotDrive import RobotDrive

import numpy as np

from NAO_RL.Driver.RobotEquipment import RobotEquipment
from NAO_RL.Tools.space_tools import apart_touch_force
from NAO_RL.Tools.utilities import getFeetPressure


class DriveFeet(RobotDrive):
    def __init__(self, drivePose: DrivePose, equipment: RobotEquipment):
        super().__init__(equipment)
        self.drivePose = drivePose

    def getFeetPose(self):
        """
        :return: 左脚、右脚的位置加姿态
        """
        pose = self.drivePose.limb_pose.getLimbPose(self.drivePose.joint_radians, to_rpy=False)
        return pose['LFoot'], pose['RFoot']

    def getFeetTouch(self):
        """
        :return: 左脚、右脚各4个部位的压力，取值[0, 25]newtons
        """
        fsv = [self.fsr[0].getValues(), self.fsr[1].getValues()]  # force sensor values

        # C语言类型处理
        for i, v in enumerate(fsv):
            fsv[i] = [v[0], v[1], v[2]]  # 变为列表

        # 接触力分解，只取地面压力
        for i, v in enumerate(fsv):
            pressure, _ = apart_touch_force(np.array(v), self.fsr[i])
            fsv[i] = list(pressure)

        left, right = getFeetPressure(fsv)
        return left, right
