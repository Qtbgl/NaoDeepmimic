import numpy as np

from nao.fk.NaoSelfPose import NaoSelfPose


class NaoKeypoint:
    # 关键点取名，其中自定义名称有Head、Foot、Toe，其余名称延用NAO官网Joints
    _keypoint_name = [
        "Head",
        "LShoulderPitch", "LElbowYaw", "LHand",
        "RShoulderPitch", "RElbowYaw", "RHand",
        "LHipYawPitch", "LKneePitch", "LFoot", "LToe",
        "RHipYawPitch", "RKneePitch", "RFoot", "RToe",
    ]

    @property
    def keypoint_name(self):  # 外部访问接口
        return self._keypoint_name

    # keypoint所属的关节点坐标系，None表示与同名关节点（或手部）重合
    _keypoint_frame = [
        "HeadPitch",
        None, None, None,
        None, None, None,
        None, None, "LAnkleRoll", "LAnkleRoll",
        None, None, "RAnkleRoll", "RAnkleRoll",
    ]
    # keypoint的所属坐标系上的位置，None表示零偏移（手部例外）
    _keypoint_offset = [
        (0, 0, 53.5),  # 头部中心向上约53.5mm
        None, None, None,
        None, None, None,
        None, None, (0.0, 0.0, -45.19), (90.0, 0.0, -45.19),  # FootHeight 45.19mm
        None, None, (0.0, 0.0, -45.19), (90.0, 0.0, -45.19),  # 脚尖向前约90mm
    ]

    def __init__(self, nao_pose: NaoSelfPose):
        self.nao_pose = nao_pose

    def getKeypoint(self, joint_rotation_radians: dict) -> dict[str, np.ndarray]:
        """
        :return: 关键点位置字典，相对于Torso坐标系，长度单位米，有关键点名字索引
        """
        keypoint = {}

        pose = self.nao_pose.getJointPoses(joint_rotation_radians)
        # 计算每一个关键点位置
        for i, name in enumerate(self._keypoint_name):
            # 获取关键点所处的坐标系
            frame = self._keypoint_frame[i]
            frame = frame if frame is not None else name
            # 获取坐标系位姿
            position, rotation = pose[frame]
            # 计算关键点相对Torso的具体位置
            if self._keypoint_offset[i] is not None:
                # 获取关键点相对位移
                offset = np.array(self._keypoint_offset[i])
                # 将相对位移映射到Torso坐标系下
                R = rotation
                t = np.dot(R, offset)
                # keypoint为坐标系位置加相对位移
                keypoint[name] = position + t
            else:
                keypoint[name] = position

        # 长度单位变为米  debugged
        for name in keypoint:
            keypoint[name] = keypoint[name] / 1000

        return keypoint
