from abc import ABC

from deepbots.supervisor import RobotSupervisorEnv

from controller import Supervisor
from nao.env.EnvAccess import EnvAccess
from nao.env.EnvMethod import EnvMethod


class HandleEnv(RobotSupervisorEnv, EnvAccess, ABC):
    def __init__(self, timestep: int):
        RobotSupervisorEnv.__init__(self, timestep)
        EnvAccess.__init__(self)  # 改用多继承
        self._env_method = EnvMethod()  # 环境内部方法接口，默认设置调用空方法

    @property
    def env_access(self) -> EnvAccess:  # 兼容以往
        return self

    @property
    def env_method(self):  # 为了设置而存在
        return self._env_method

    @env_method.setter
    def env_method(self, value: EnvMethod):  # 外部类设置接口
        self._env_method = value

    def step(self, action):
        self.apply_action(action)
        self.do_simulation()
        self._count_step += 1  # 运行action后追加count_step
        return (
            self.get_observations(),
            self.get_reward(action),
            self.is_done(),
            self.get_info(),
        )

    def reset(self):
        """
        episode自定义重置方法，后调用父类重置
        """
        self._epi_time = 0  # 1.episode时间重置
        self._count_step = 0  # 1.episode步数重置
        self._episode += 1  # 1.episode序号更新

        if self._episode > 0 and self.env_method.just_continue():  # 最初一次episode一定重置
            # 4.获取当前观察
            obs = self.get_observations()
        else:
            # 2.Webots环境重置
            self.simulationReset()
            self.simulationResetPhysics()
            self._sim_time = 0

            # 3.自定义整合重置方法
            init = self.env_method.reset()
            init = self.default_initial_reset() if init is None else init
            for delta in init:
                super(Supervisor, self).step(delta)  # 1.自定义初始设置
                self._sim_time += delta  # 2.更新时间值
                self.env_method.update(delta)  # 3.更新自定义状态

            # 4.获取默认观察
            obs = self.get_default_observation()

        return obs

    def default_initial_reset(self):
        yield int(self.getBasicTimeStep())

    def do_simulation(self):
        """
        模拟运行时间
        """
        rl = self.timestep
        delta = int(self.getBasicTimeStep())  # 时间分片=webots最短步长
        t = rl / delta
        assert t == int(t), "Rl决策间隔应为Webots时间步长的整倍数"
        for i in range(int(t)):
            # 1.模拟时间推进
            if super(Supervisor, self).step(delta) == -1:
                exit()

            # 2.更新时间值
            self._epi_time += delta
            self._sim_time += delta

            # 3.更新自定义状态
            self.env_method.update(delta)
