from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotEquipment import RobotEquipment


class DrivePose(RobotDrive):
    def __init__(self, equipment: RobotEquipment):
        super().__init__(equipment)

    @property
    def joint_radians(self):
        return self.get_joint_radians()  # 关节角度字典，关节名索引

    @property
    def joint_number(self):
        return len(self.ps)  # 关节传感器个数

    def get_joint_radians(self):
        joint_radians = {}
        # 获取注册过的电机转角
        for name, sensor in self.ps.items():
            radian = sensor.getValue()
            joint_radians[name] = radian

        return joint_radians

