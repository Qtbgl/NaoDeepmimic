import numpy as np
from scipy.spatial.transform import Rotation


class NaoSelfPose:
    joint_names = [
        "Torso", "HeadYaw", "HeadPitch",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"
    ]
    # 层次结构，子关节下标，与joint_names对应
    joint_chains = [[1, 3, 9, 15, 21], 2, -1, 4, 5, 6, 7, 8, -1, 10, 11, 12, 13, 14, -1, 16, 17, 18, 19, 20, -1, 22, 23, 24, 25, 26, -1]

    # 关节在零姿态下的偏移，每个元素存xyz位移（计划从文件中读取）
    joint_offsets: np.ndarray
    # 关节的旋转轴（计划从文件中读取）
    rotation_axis: np.ndarray

    def __init__(self, load_data_fun):
        self.joint_offsets, self.rotation_axis = load_data_fun(self.joint_names)

    def getJointPoses(self, rotation_radians: dict) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """
        :param rotation_radians:
        :return: 各关节相对Torso的位姿
        """
        return_pose = {}

        # 暂时不限制左右髋关节自由度（包括智能体动作控制那一边）
        # # LHipYawPitch角度有优先级
        # if "LHipYawPitch" in rotation_radians:
        #     rotation_radians["RHipYawPitch"] = rotation_radians["LHipYawPitch"]
        # elif "RHipYawPitch" in rotation_radians:
        #     rotation_radians["LHipYawPitch"] = rotation_radians["RHipYawPitch"]

        # 获取Torso下一级的关节/关节下标
        joint_tops = self.joint_chains[self.joint_names.index('Torso')]
        for top in joint_tops:
            # 默认Torso的位置与姿态
            parent_position = np.zeros(3)
            parent_rotation = np.eye(3)

            i = top
            while i != -1:  # 逐级更新每一个关节坐标系
                # 入参关节旋转角度
                rotation_radian = rotation_radians.get(self.joint_names[i], 0.0)
                # 获取关节旋转轴
                axis = self.rotation_axis[i, :]
                # 轴角表示相对旋转
                rotation = Rotation.from_rotvec(axis * rotation_radian)
                # 计算当前关节姿态，旋转相对与上一次坐标系，右乘  debugged
                rotation = rotation.as_matrix()
                current_rotation = np.dot(parent_rotation, rotation)
                # 获取关节偏移量
                offset = self.joint_offsets[i, :]
                # 计算当前关节位置
                current_position = parent_position + np.dot(parent_rotation, offset)
                # 保存位姿数据
                return_pose[self.joint_names[i]] = (current_position, current_rotation)

                i = self.joint_chains[i]  # 转到下一个关节/关节下标
                parent_position = current_position  # debugged
                parent_rotation = current_rotation

        return return_pose
