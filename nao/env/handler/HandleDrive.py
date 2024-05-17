from nao.env.EnvMethod import EnvMethod
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.RobotEquipment import RobotEquipment
from nao.env.WebotsEnv import WebotsEnv


class HandleDrive(EnvMethod):
    def __init__(self, env: WebotsEnv):
        """
        基类中唯一创建NAO设备实例，和最基本的驱动类
        """
        self.equipment = RobotEquipment(env)  # 单例模式
        self.base = RobotDrive(self.equipment)  # RobotDrive多例依赖于RobotEquipment
