from nao.env.WebotsEnv import WebotsEnv
from nao.env.driver.impl.DriveMotion import DriveMotion
from nao.env.handler.impl.HandleDrive104 import HandleDrive104
from nao.env.tool import motion_tools
from nao.env.tool.collections.KeyQueue import KeyQueue
from nao.mo.MotionCSV import MotionCSV


class HandleDrive1041(HandleDrive104):
    def __init__(self, env: WebotsEnv, motion_file: str):
        super().__init__(env)
        self.env = env.env_access
        self.supervisor = env
        self.driveMotion = DriveMotion(self.equipment)
        # 假设模仿走路动作
        self.motion = MotionCSV(motion_file)
        self.bind_hip_motors = True
        # 用于奖励动作对比
        self.kqSample = KeyQueue(10)
        self.kqAgent = KeyQueue(10)
        self.kqReward = KeyQueue(max_size=10)
        # 用于奖励轨迹对比
        motion_tools.fit(env, self.base)
        self.effector, self.mass_center = motion_tools.get_traj(self.motion)

    @property
    def max_epi_time(self):
        return 10_000  # 每个动作最多10秒

    def reset(self):
        """
        均为重置数据，调用先后无关
        """
        self.driveFallen.clean()
        self.driveLimbVel2.reset()  # 来自父类
        self.kqSample.clear()
        self.kqAgent.clear()
        self.kqReward.clear()

        return self.init_move()

    def init_move(self):
        for i in range(3):
            motion = self.sample_angles()
            self.apply_motion(motion)
            yield 40

    def sample_angles(self):
        # 当前时间的示例动作
        motion, _, _ = self.motion.getMotionByTime(self.env.sim_time)
        return motion

    def apply_motion(self, motion):
        self.kqAgent.set(self.env.sim_time, motion)  # 记录输出动作
        self.kqSample.set(self.env.sim_time, self.sample_angles())  # 记录示例
        # 设置电机角度
        if self.bind_hip_motors:
            motion["RHipYawPitch"] = motion["LHipYawPitch"]  # 电机联动

        motion_tools.setMotor(motion)

    def time_before(self, sim_time):
        """
        查询动作输出时间
        """
        times = self.kqAgent.keys()
        assert times[0] < sim_time
        i = len(times) - 1
        while times[i] >= sim_time:  # 获取第一个小于此时间的
            i -= 1

        return times[i]
