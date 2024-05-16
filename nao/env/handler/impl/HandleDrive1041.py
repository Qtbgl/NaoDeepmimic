from nao.env.WebotsEnv import HandleEnv
from nao.mo.MotionCSV import MotionCSV
from nao.env.handler.impl.HandleDrive104 import HandleDrive104
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.impl.DriveMotion import DriveMotion
from nao.env.tool import space_tools
from nao.env.tool.collections.KeyQueue import KeyQueue
from nao.env.tool.space_tools import get_effector_position, get_mass_center
from controller import Supervisor


class HandleDrive1041(HandleDrive104):
    def __init__(self, env: HandleEnv, motion_file: str):
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
        self.effector = {}
        self.mass_center = {}
        self._get_traj(env, self.motion)

    def _get_traj(self, supervisor: Supervisor, motion: MotionCSV):
        space_tools.fit(supervisor)  # debug
        supervisor.simulationReset()
        supervisor.simulationResetPhysics()
        super(Supervisor, supervisor).step()  # 重置世界后更新
        time = 0
        basic = int(supervisor.getBasicTimeStep())
        print('(HandleDrive1041)正在采集轨迹')
        while not motion.is_end(time):
            # 记录效应器的世界位置
            self.effector[time] = get_effector_position(['LLeg', 'RLeg', 'LArm', 'RArm'])
            # 记录质心的世界位置
            self.mass_center[time] = get_mass_center()
            # 更新动作下一步
            angles, _, _ = motion.getMotionByTime(time)
            setMotor(angles, self.base)
            if super(Supervisor, supervisor).step(basic) == -1:
                break

            time += basic

        print('(HandleDrive1041)采集完成, time', time)
        supervisor.simulationReset()
        supervisor.simulationResetPhysics()
        super(Supervisor, supervisor).step()


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
            motion = self.sample_motion()
            self.apply_motion(motion)
            yield 40

    def sample_motion(self):
        # 当前时间的示例动作
        motion, _, _ = self.motion.getMotionByTime(self.env.sim_time)
        return motion

    def apply_motion(self, motion):
        self.kqAgent.set(self.env.sim_time, motion)  # 记录输出动作
        self.kqSample.set(self.env.sim_time, self.sample_motion())  # 记录示例
        # 设置电机角度
        if self.bind_hip_motors:
            motion["RHipYawPitch"] = motion["LHipYawPitch"]  # 电机联动

        setMotor(motion, self.base)

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


def setMotor(motion: dict, drive: RobotDrive):
    # 设置电机转角
    for name in drive.motors:  # 只控制已注册的电机
        motor = drive.motors[name]
        angle = motion.get(name, 0.0)
        motor.setPosition(angle)
