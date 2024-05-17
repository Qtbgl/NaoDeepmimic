from controller import Supervisor
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.tool import space_tools
from nao.env.tool.space_tools import get_effector_position, get_mass_center
from nao.mo.MotionCSV import MotionCSV


_supervisor: Supervisor
_drive: RobotDrive


def fit(supervisor: Supervisor, drive: RobotDrive):
    """
    在调用工具方法前注入其依赖
    """
    global _supervisor
    global _drive
    _supervisor = supervisor
    _drive = drive


def get_traj(motion: MotionCSV):
    """
    :return: effector, mass_center随着动作的轨迹
    """
    space_tools.fit(_supervisor)  # debug
    _supervisor.simulationReset()
    _supervisor.simulationResetPhysics()
    super(Supervisor, _supervisor).step()  # 重置世界后更新
    time = 0
    basic = int(_supervisor.getBasicTimeStep())
    print('正在采集轨迹')
    effector = {}
    mass_center = {}
    while not motion.is_end(time):
        # 记录效应器的世界位置
        effector[time] = get_effector_position(['LLeg', 'RLeg', 'LArm', 'RArm'])
        # 记录质心的世界位置
        mass_center[time] = get_mass_center()
        # 更新动作下一步
        angles, _, _ = motion.getMotionByTime(time)
        setMotor(angles)
        if super(Supervisor, _supervisor).step(basic) == -1:
            break

        time += basic

    print('采集完成 time', time)
    _supervisor.simulationReset()
    _supervisor.simulationResetPhysics()
    super(Supervisor, _supervisor).step()
    
    return effector, mass_center


def setMotor(motion: dict):
    # 设置电机转角
    for name in _drive.motors:  # 只控制已注册的电机
        motor = _drive.motors[name]
        angle = motion.get(name, 0.0)
        motor.setPosition(angle)
