import numpy as np

from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.RobotEquipment import RobotEquipment


class DriveMotion(RobotDrive):
    def __init__(self, equipment: RobotEquipment):
        super().__init__(equipment)
        self.keypoint_chains = [
            ["LShoulderPitch", "LElbowYaw", "LHand"],
            ["RShoulderPitch", "RElbowYaw", "RHand"],
            ["LHipYawPitch", "LKneePitch", "LFoot"],
            ["RHipYawPitch", "RKneePitch", "RFoot"], ]

    def getPoint(self, angles: dict):
        keypoint = self.nao_keypoint.getKeypoint(angles)
        points = []
        chains = self.keypoint_chains
        # 获取朝向向量
        for names in chains:
            point1 = keypoint[names[1]]
            point2 = keypoint[names[2]]
            points += list(point1) + list(point1)  # 展开列表
        return points

    def getArrow(self, angles: dict):  # 表示动作特征最好
        keypoint = self.nao_keypoint.getKeypoint(angles)
        arrows = []
        chains = self.keypoint_chains
        # 获取朝向向量
        for names in chains:
            arrow1 = keypoint[names[1]] - keypoint[names[0]]
            arrow2 = keypoint[names[2]] - keypoint[names[1]]
            arrow1 /= np.linalg.norm(arrow1)  # 单位向量
            arrow2 /= np.linalg.norm(arrow2)
            arrows += list(arrow1) + list(arrow2)  # 展开列表
        return arrows

    def getOrient(self, angles: dict):
        keypoint = self.nao_keypoint.getKeypoint(angles)
        arrows = []
        chains = self.keypoint_chains
        # 获取朝向向量
        for names in chains:
            arrow1 = keypoint[names[1]] - keypoint[names[0]]
            arrow2 = keypoint[names[2]] - keypoint[names[1]]
            arrows += [arrow1, arrow2]
        # 计算极坐标朝向
        orient = []
        for arrow in arrows:
            phi, theta = cartesian_to_polar(*arrow)  # 可表示单位向量
            orient += [phi, theta]
        return orient


def cartesian_to_polar(x, y, z):
    rho = np.sqrt(x ** 2 + y ** 2)  # x,y的长度
    phi = np.arctan2(y, x)
    theta = np.arctan2(z, rho)  # π/2朝上-> 0水平-> -π/2朝下
    return phi, theta
