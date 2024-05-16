from nao.env.driver.RobotEquipment import RobotEquipment


class RobotDrive:
    """
    激活和访问设备
    """
    def __init__(self, equipment: RobotEquipment):
        self.acc = equipment.acc
        self.gyro = equipment.gyro
        self.imu = equipment.imu
        self.motors = equipment.motors
        self.ps = equipment.ps
        self.fsr = equipment.fsr
        self.nao_pose = equipment.nao_pose
        self.limb_pose = equipment.limb_pose
        self.nao_keypoint = equipment.nao_keypoint
        self.nao_limb = equipment.nao_limb

    def enable(self, p):
        """
        :param p: 传感器采样时间
        """
        # 激活Equipment类的传感器
        self.acc.enable(p)
        self.gyro.enable(p)
        self.imu.enable(p)
        for sensor in self.ps.values():
            sensor.enable(p)
        for sensor in self.fsr:
            sensor.enable(p)
