from nao.env.WebotsEnv import HandleEnv
from nao.env.driver.impl.DriveImu import DriveImu
from nao.env.driver.impl.DriveLimbVel2 import DriveLimbVel2
from nao.env.driver.impl.DriveTorso import DriveTorso
from nao.env.handler.impl.HandleDrive1 import HandleDrive1


class HandleDrive104(HandleDrive1):

    def __init__(self, env: HandleEnv):
        super().__init__(env)
        self.driveLimbVel2 = DriveLimbVel2(self.equipment, env.env_access, self.drivePose)
        self.driveImu = DriveImu(self.equipment)
        self.driveTorso = DriveTorso(env, self.equipment)

    def update(self, delta):
        """
        先调用父类清空缓存，再更新数据
        """
        # print('update', self.driveLimbVel2.env.sim_time)
        self.driveFallen.clean()
        self.driveLimbVel2.update()

    def reset(self):
        """
        均为重置数据，调用先后无关
        """
        self.driveFallen.clean()
        self.driveLimbVel2.reset()
        return self.init_delay()

    def init_delay(self):
        # 初始延迟用于传感器采样，单位ms
        # yield 100
        for i in range(5):
            yield 20
