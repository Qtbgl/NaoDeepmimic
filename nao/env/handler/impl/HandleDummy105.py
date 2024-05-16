import numpy as np
from gym.spaces import Box

from nao.env.EnvAccess import EnvAccess
from nao.env.driver.impl.DriveFallen import DriveFallen
from nao.env.driver.impl.DrivePose import DrivePose
from nao.env.handler.impl.HandleAction301 import HandleAction301
from nao.env.handler.impl.HandleDrive1041 import HandleDrive1041
from nao.env.handler.impl.HandleObservation401 import HandleObservation401
from nao.env.handler.impl.HandleDummy1 import HandleDummy1
from nao.env.tool.space_tools import get_effector_position, get_mass_center


class HandleDummy105(HandleDummy1):
    """
    继承分支使用HandleAction301和HandleObservation401
    """
    @property
    def observation_space(self):
        # 观察空间加入动作信息
        obs_space = self.obs.observation_space
        act_space = self.act.action_space
        # 新的观察空间
        obs_low = list(obs_space.low) + list(act_space.low)
        obs_high = list(obs_space.high) + list(act_space.high)
        return Box(low=np.array(obs_low), high=np.array(obs_high), dtype=np.float64)

    def __init__(self, act: HandleAction301, obs: HandleObservation401, env: EnvAccess,
                 drive: HandleDrive1041, driveFallen: DriveFallen, drivePose: DrivePose):
        super().__init__(act, env)
        # 依赖注入
        self.obs = obs
        self.drive = drive
        self.driveFallen = driveFallen
        self.drivePose = drivePose

    def get_observations(self):
        # 更新和获取示例动作
        sample = self.getActionInOrder(self.drive.sample_motion())
        obs = self.obs.get_observations()
        obs = obs + sample
        return obs

    def get_default_observation(self):
        # 观察状态加动作信息
        sample = self.getActionInOrder(self.drive.sample_motion())
        # sample = self.add_noise(sample)
        obs = self.obs.get_default_observation() + sample
        return obs

    def apply_action(self, action):
        # 应用实际动作
        motion = self.getMotionByOrder(action)
        self.drive.apply_motion(motion)

    def is_truncated(self):
        is_end = self.drive.motion.is_end(self.env.sim_time + 100)  # 模拟到结束
        # is_end = self.env.epi_time >= self.drive.max_epi_time   # 固定最长时间
        return is_end

    def is_terminated(self):
        is_fallen = self.driveFallen.is_fallen  # early termination
        return is_fallen

    def get_reward(self, action):
        # 计算示例动作与输出的差异
        last_time = self.drive.time_before(self.env.sim_time)
        sample = self.drive.kqSample.get(last_time)
        agent = self.drive.kqAgent.get(last_time)  # radian角度
        r_p = get_reward_from_error(sample, agent, 1, scalar_error)  # decay_rate=2

        # 计算示例速度与输出速度的差异
        t1 = self.drive.time_before(self.env.sim_time)
        t0 = self.drive.time_before(t1)
        t = (t1 - t0) / 1000  # 单位秒
        sample1 = self.drive.kqSample.get(t1)
        sample0 = self.drive.kqSample.get(t0)
        agent1 = self.drive.kqAgent.get(t1)
        agent0 = self.drive.kqAgent.get(t0)
        # 计算rad/s角速度
        sampleV = {}
        agentV = {}
        for name in sample1:
            sampleV[name] = (sample1[name] - sample0[name]) / t
            agentV[name] = (agent1[name] - agent0[name]) / t

        r_v = get_reward_from_error(sampleV, agentV, 0.03, scalar_error)  # decay_rate=0.1

        # 计算末端效应器的差异
        now = get_effector_position(['LLeg', 'RLeg', 'LArm', 'RArm'])
        refer = self.drive.effector[self.env.sim_time]
        r_e = get_reward_from_error(refer, now, 10, vector_error)  # decay_rate=10 DeepMimic论文中有问题

        # 计算质心的差异
        now = {'mc': get_mass_center()}
        refer = {'mc': self.drive.mass_center[self.env.sim_time]}
        r_c = get_reward_from_error(refer, now, 40, vector_error)  # decay_rate=40 DeepMimic论文中有问题

        # print('r_p', r_p, 'r_v', r_v, 'r_e', r_e, 'r_c', r_c)
        reward_info = {'r_p': r_p, 'r_v': r_v, 'r_e': r_e, 'r_c': r_c}
        self.drive.kqReward.set(self.env.count_step, reward_info)

        reward = 0.65 * r_p + 0.1 * r_v + 0.15 * r_e + 0.1 * r_c
        return reward

    def get_info(self):
        info = super().get_info()
        info.update(self.drive.kqReward.get(self.env.count_step))
        return info


def scalar_error(a, b):
    return (a - b) ** 2  # Scalar值之差平方


def vector_error(v, u):
    return np.sum((v - u) ** 2)  # 几何点距离平方


def get_reward_from_error(sample_out: dict, agent_out: dict, decay_rate: float, calc_error):
    """
    :param sample_out: 字典同键
    :param agent_out: 字典同键
    :param decay_rate: 损失-奖励的衰减率>0
    :param calc_error: 计算偏差方法
    :return: 奖励从1一直衰减到0
    """
    assert decay_rate > 0, '衰减率>0'
    sum_error = 0
    for name in sample_out:
        error = calc_error(agent_out[name], sample_out[name])
        sum_error += error
    reward = np.exp(-decay_rate * sum_error)
    return reward
