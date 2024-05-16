import numpy as np
from scipy.spatial.transform import Rotation

from nao.fk.NaoSelfPose import NaoSelfPose


class NaoSelfLimbPose:
    # limb名称部分来源 http://doc.aldebaran.com/2-8/family/nao_technical/links_naov6.html
    _limb_names = [
        'LUpperArm', 'LLowerArm', 'LHand',
        'RUpperArm', 'RLowerArm', 'RHand',
        'LThigh', 'LTibia', 'LFoot',
        'RThigh', 'RTibia', 'RFoot',
    ]

    @property
    def limb_names(self):
        return self._limb_names

    @property
    def get_limb_number(self):
        return len(self.limb_locations[0])

    limb_locations = [[
        # 姿态上对应的关节点
        'LShoulderRoll', 'LElbowRoll', 'LWristYaw',
        'RShoulderRoll', 'RElbowRoll', 'RWristYaw',
        'LHipPitch', 'LKneePitch', 'LAnkleRoll',
        'RHipPitch', 'RKneePitch', 'RAnkleRoll'], [
        # limb对应的位置点，相对于关节坐标系，None表示默认下一个关节点
        None, None, None,
        None, None, None,
        None, None, (0.0, 0.0, -45.19),  # FootHeight 45.19mm
        None, None, (0.0, 0.0, -45.19),
    ]]
    # 具体limb的位置点
    limb_offsets: np.ndarray

    def __init__(self, nao_pose: NaoSelfPose):
        self.nao_pose = nao_pose

        # 位置对应点程序检查
        limb_r_points = self.limb_locations[0]
        limb_p_points = self.limb_locations[1]

        # 检查对应点在一根连杆上（已省略）
        assert len(limb_r_points) == len(limb_p_points) == len(self.limb_names)

        # 记录limb的位置点
        self.limb_offsets = np.zeros((len(limb_p_points), 3))
        for i in range(0, len(limb_p_points)):
            # 记录以上下级关系定义的点
            if limb_p_points[i] is None:
                limb_r_index = nao_pose.joint_names.index(limb_r_points[i])
                index = nao_pose.joint_chains[limb_r_index]
                # 复制位置点对应的一行
                self.limb_offsets[i, :] = nao_pose.joint_offsets[index, :]
            else:
                self.limb_offsets[i, :] = limb_p_points[i]

    def getLimbPose(self, joint_rotation_radians: dict, to_rpy=True) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """
        :param joint_rotation_radians:
        :param to_rpy: 姿态是否以rpy欧拉角返回
        :return: 各个limb的位置与姿态
        """
        return_pose = {}

        # 利用nao姿态模型
        pose = self.nao_pose.getJointPoses(joint_rotation_radians)
        # 解析位姿数据
        for i, name in enumerate(self.limb_locations[0]):
            joint_position, rotation = pose[name]
            # 计算limb的位置
            offset = self.limb_offsets[i, :]
            position = joint_position + np.dot(rotation, offset)
            if to_rpy:
                # 以rpy欧拉角表示
                rotation = Rotation.from_matrix(rotation)
                # x-y-z外在旋转，或ZYX内禀旋转
                rotation = rotation.as_euler('xyz', degrees=False)
            # 保存位姿数据
            return_pose[self.limb_names[i]] = (position, rotation)

        return return_pose
