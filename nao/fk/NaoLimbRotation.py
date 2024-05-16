import numpy as np

from nao.fk.NaoSelfPose import NaoSelfPose


class NaoLimbRotation:
    # limb名称部分来源 http://doc.aldebaran.com/2-8/family/nao_technical/links_naov6.html
    _limb_name = [
        'Head',
        'LUpperArm', 'LLowerArm', 'LHand',
        'RUpperArm', 'RLowerArm', 'RHand',
        'LThigh', 'LTibia', 'LFoot',
        'RThigh', 'RTibia', 'RFoot',
    ]

    @property
    def limb_names(self):
        return self._limb_name

    # limb同姿态的关节点坐标系
    _limb_frame = [
        'HeadPitch',
        'LShoulderRoll', 'LElbowRoll', 'LWristYaw',
        'RShoulderRoll', 'RElbowRoll', 'RWristYaw',
        'LHipPitch', 'LKneePitch', 'LAnkleRoll',
        'RHipPitch', 'RKneePitch', 'RAnkleRoll',
    ]

    def __init__(self, nao_pose: NaoSelfPose):
        self.nao_pose = nao_pose

    def getLimbRotation(self, joint_rotation_radians: dict) -> dict[str, np.ndarray]:
        """
        :return: limb相对Torso旋转字典，长度单位米，由limb名称名字索引
        """
        limb_rot = {}
        # 利用nao姿态模型
        pose = self.nao_pose.getJointPoses(joint_rotation_radians)
        # 计算每一个limb的姿态
        for i, name in enumerate(self._limb_name):
            _, rotation = pose[self._limb_frame[i]]
            limb_rot[name] = rotation

        return limb_rot
