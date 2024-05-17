from nao.env.WebotsEnv import WebotsEnv
from nao.env.driver.impl.DriveFallen import DriveFallen
from nao.env.driver.impl.DriveFeet import DriveFeet
from nao.env.driver.impl.DrivePose import DrivePose
from nao.env.handler.HandleDrive import HandleDrive


class HandleDrive1(HandleDrive):
    """
    优先注册要随时clean的驱动类
    """
    def __init__(self, env: WebotsEnv):
        super().__init__(env)
        # 注册任务需要的驱动类
        self.drivePose = DrivePose(self.equipment)
        self.driveFeet = DriveFeet(self.drivePose, self.equipment)
        self.driveFallen = DriveFallen(env, self.equipment)

    def update(self, delta):  # 运行模拟时清空
        self.driveFallen.clean()

    def reset(self):  # episode重置时清空
        self.driveFallen.clean()
