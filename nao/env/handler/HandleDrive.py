from nao.env.EnvMethod import EnvMethod
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.RobotEquipment import RobotEquipment
from nao.env.WebotsEnv import HandleEnv


class HandleDrive(EnvMethod):
    def __init__(self, env: HandleEnv):
        """
        基类中唯一创建NAO设备实例，和最基本的驱动类
        """
        self.equipment = RobotEquipment(env)  # 单例模式
        self.base = RobotDrive(self.equipment)  # RobotDrive多例依赖于RobotEquipment
