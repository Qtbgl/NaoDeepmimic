import numpy as np

from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotDriveImpl.DriveTorso import DriveTorso
from NAO_RL.Driver.RobotEquipment import RobotEquipment
from NAO_RL.Handler.EnvAccess import EnvAccess


class DriveTorsoTraj(RobotDrive):
    def __init__(self, equipment: RobotEquipment, driveTorso: DriveTorso, env: EnvAccess):
        super().__init__(equipment)
        self.driveTorso = driveTorso
        self.env = env
        # 记录移动状态
        self.moving = {}

    class TrajItem:
        def __init__(self, v: np.ndarray, angular_v: np.ndarray):
            self.v = v
            self.angular_v = angular_v

    def reset(self):
        self.moving = {}

    def update(self):
        sim_ms = self.env.sim_time
        v, a = self.driveTorso.get_velocity_by_world_in_torso()
        self.moving[sim_ms] = self.TrajItem(v, a)

    def get_recent_traj(self, current_delta) -> list[TrajItem]:
        """
        :param current_delta: 与当前的时间差，单位毫秒
        :return: 离当前时间近的数据靠前
        """
        moving = self.moving
        now = self.env.sim_time
        traj = []
        times = sorted(list(moving.keys()), reverse=True)
        for time in times:  # 按次序变遍历
            if now - time <= current_delta:  # 小于时间差
                traj.append(moving[time])

        return traj
