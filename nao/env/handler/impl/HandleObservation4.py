import numpy as np
from gym.spaces import Box
from scipy.spatial.transform import Rotation

from nao.env.driver.impl.DriveLimbVel2 import DriveLimbVel2
from nao.env.driver.impl.DrivePose import DrivePose
from nao.env.handler.HandleObservation import HandleObservation


class HandleObservation4(HandleObservation):
    """
    NAO机器人自身的状态，不涉及环境
    """
    limb_pos_min = [-1, -1, -1] * 15  # 身体关键点相对于Torso的位置
    limb_pos_max = [1, 1, 1] * 15
    limb_rot_min = [-np.inf, -np.inf, -np.inf, -np.inf] * 13  # 四元数表示相对Torso旋转
    limb_rot_max = [np.inf, np.inf, np.inf, np.inf] * 13
    limb_l_v_min = [-np.inf, -np.inf, -np.inf] * 11  # 身体关节点相对于Torso的线速度
    limb_l_v_max = [np.inf, np.inf, np.inf] * 11
    limb_a_v_min = [-np.inf, -np.inf, -np.inf] * 13  # limb相对于Torso的角速度
    limb_a_v_max = [np.inf, np.inf, np.inf] * 13
    # 速度一直为零的关键点
    keypoint_v_exclude = ["LShoulderPitch", "RShoulderPitch", "LHipYawPitch", "RHipYawPitch"]

    @property
    def observation_space(self):
        obs_low = np.array(
            self.limb_pos_min + self.limb_rot_min + self.limb_l_v_min + self.limb_a_v_min, dtype=np.float64)
        obs_high = np.array(
            self.limb_pos_max + self.limb_rot_max + self.limb_l_v_max + self.limb_a_v_max, dtype=np.float64)
        return Box(low=obs_low, high=obs_high, dtype=np.float64)

    def __init__(self, drivePose: DrivePose, driveLimbVel2: DriveLimbVel2):
        self.drivePose = drivePose
        self.driveLimbVel2 = driveLimbVel2

    def get_observations(self):
        limb_pos = self.getLimbPosition()
        limb_rot = self.getLimbRotation()
        limb_l_v = self.getLimbLinearVel()
        limb_a_v = self.getLimbAngularVel()
        obs = limb_pos + limb_rot + limb_l_v + limb_a_v
        return obs

    def get_default_observation(self):
        return self.get_observations()  # 初次已有运动情况

    def getLimbPosition(self):
        # 获取关键点相对躯干的位置
        joint_angles = self.drivePose.joint_radians
        keypoint = self.drivePose.nao_keypoint.getKeypoint(joint_angles)
        position = []
        # 展开为列表
        for name in keypoint:
            position += list(keypoint[name])

        return position

    def getLimbRotation(self):
        # 获取limb相对躯干的姿态
        joint_angles = self.drivePose.joint_radians
        limb_R = self.drivePose.nao_limb.getLimbRotation(joint_angles)
        rotation = []
        # 转换为四元数，并加入列表
        for name in limb_R:
            R = limb_R[name]
            quat = Rotation.from_matrix(R).as_quat()
            rotation += list(quat)

        return rotation

    def getLimbLinearVel(self):
        limb_l_v = []
        # 获取相对Torso速度
        v = self.driveLimbVel2.local_linear_v_2  # 优化速度计算方式
        for name in v:
            if name not in self.keypoint_v_exclude:
                limb_l_v += list(v[name])

        return limb_l_v

    def getLimbAngularVel(self):
        limb_a_v = []
        a = self.driveLimbVel2.local_angular_v_2  # 优化速度计算方式
        for name in a:
            limb_a_v += list(a[name])

        return limb_a_v
