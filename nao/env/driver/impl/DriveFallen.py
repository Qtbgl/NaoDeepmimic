import numpy as np

from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotEquipment import RobotEquipment
from controller import Supervisor


class DriveFallen(RobotDrive):
    def __init__(self, supervisor: Supervisor, equipment: RobotEquipment):
        super().__init__(equipment)
        self.supervisor = supervisor

        # 更新状态
        self.unload = True
        self.__is_fallen = None

    @property
    def is_fallen(self):
        if self.unload:
            self.__is_fallen = self.get_is_fallen()
            self.unload = False

        return self.__is_fallen

    def clean(self):
        self.unload = True

    def get_is_fallen(self):
        # 使用Supervisor读取位姿
        head_T_world = np.array(self.supervisor.getFromDef("Head").getPose()).reshape((4, 4))
        torso_T_world = np.array(self.supervisor.getFromDef("Torso").getPose()).reshape((4, 4))
        l_arm_T_world = np.array(self.supervisor.getFromDef("LArm").getPose()).reshape((4, 4))
        r_arm_T_world = np.array(self.supervisor.getFromDef("RArm").getPose()).reshape((4, 4))
        # 获取世界坐标系下z轴位移
        z = [head_T_world[2, 3],
             torso_T_world[2, 3],
             l_arm_T_world[2, 3],
             r_arm_T_world[2, 3]]

        # 判断摔倒
        # 检测躯干手臂或头部任何一点接近地面（低于10cm）
        if min(z) < 0.1:
            return True
        else:
            return False
