import numpy as np
from gym.spaces import Box

from nao.env.driver.impl.DriveFeet import DriveFeet
from nao.env.driver.impl.DriveImu import DriveImu
from nao.env.driver.impl.DriveLimbVel2 import DriveLimbVel2
from nao.env.driver.impl.DrivePose import DrivePose
from nao.env.driver.impl.DriveTorso import DriveTorso
from nao.env.handler.impl.HandleObservation4 import HandleObservation4
from nao.env.tool.utilities import normalize_to_range


class HandleObservation401(HandleObservation4):
    """
    HandleObservation4基础上，加入相对环境信息
    """
    torso_rot_min = [-np.inf, -np.inf, -np.inf, -np.inf]  # 四元数表示Torso相对大地的旋转
    torso_rot_max = [np.inf, np.inf, np.inf, np.inf]
    torso_a_v_min = [-np.inf] * 3  # 躯干实际角速度，表示在Torso坐标系
    torso_a_v_max = [np.inf] * 3
    feet_touch_min = [0, 0, 0, 0] * 2  # 压力左右脚
    feet_touch_max = [1, 1, 1, 1] * 2

    @property
    def observation_space(self):
        obs_low = np.array(
            self.limb_pos_min + self.limb_rot_min + self.limb_l_v_min + self.limb_a_v_min +
            self.torso_rot_min + self.torso_a_v_min + self.feet_touch_min, dtype=np.float64)
        obs_high = np.array(
            self.limb_pos_max + self.limb_rot_max + self.limb_l_v_max + self.limb_a_v_max +
            self.torso_rot_max + self.torso_a_v_max + self.feet_touch_max, dtype=np.float64)
        return Box(low=obs_low, high=obs_high, dtype=np.float64)

    def __init__(self, drivePose: DrivePose, driveLimbVel2: DriveLimbVel2,
                 driveImu: DriveImu, driveFeet: DriveFeet, driveTorso: DriveTorso):
        super().__init__(drivePose, driveLimbVel2)
        self.driveImu = driveImu
        self.driveFeet = driveFeet
        self.driveTorso = driveTorso

    def get_observations(self):
        obs = super().get_observations()
        torso_rot = self.getTorsoImuRotation()
        torso_a_v = self.getTorsoAngularVel()
        feet_touch = self.getFeetTouches()
        obs = obs + torso_rot + torso_a_v + feet_touch
        return obs

    def get_default_observation(self):
        return self.get_observations()  # 因为父类也如此

    def getTorsoImuRotation(self):
        """
        用传感器获取，需要采样
        """
        R = self.driveImu.torsoRotation
        torso_rot = R.as_quat()  # 转换为四元数
        return list(torso_rot)

    def getTorsoAngularVel(self):
        _, a_torso = self.driveTorso.get_velocity_by_world_in_torso()
        # 展开为列表
        a = list(a_torso)
        return a

    def getFeetTouches(self):
        """
        :return: 双脚各四个部位的压力
        """
        left, right = self.driveFeet.getFeetTouch()
        touches = left + right
        # 归一化足部压力
        feet = [normalize_to_range(touch, 0, self.each_touch_max, 0, 1, clip=True)
                for touch in touches]
        return feet
