import numpy as np

from nao.env.WebotsEnv import HandleEnv
from nao.env.handler.getter import get_handlers


class WebotsNao(HandleEnv):
    def __init__(self, **arguments):
        super().__init__(timestep=40)  # RL决策时间间隔 40ms

        # 机器人驱动类
        drive, self.obs, self.act, self.rew = get_handlers(self, arguments)
        self.env_method = drive

        self.action_space = self.act.action_space
        self.observation_space = self.obs.observation_space

        basic = int(self.getBasicTimeStep())
        drive.base.enable(basic)
        print('The Webots basic timestep is', basic)

        print(f'已注册电机: {len(drive.base.motors)}个')
        print('action_space.shape', self.action_space.shape[0])
        print('observation_space.shape', self.observation_space.shape[0])

        print('The RL timestep is', self.timestep, 'ms')

    def get_observations(self):
        return np.array(self.obs.get_observations())

    def get_default_observation(self):
        # 返回默认观察
        return np.array(self.obs.get_default_observation())

    def is_done(self):
        return self.rew.is_done()

    def apply_action(self, action):
        self.act.apply_action(list(action))

    def get_reward(self, action):
        return self.rew.get_reward(action)

    def get_info(self):
        return self.obs.get_info()

    def render(self, mode="human"):
        pass
