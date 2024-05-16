import numpy as np

from nao.env.driver.impl.DriveMotion import DriveMotion, cartesian_to_polar
from nao.env.driver.RobotEquipment import RobotEquipment


class DriveMotionExtra(DriveMotion):
    def __init__(self, equipment: RobotEquipment):
        super().__init__(equipment)

    @property
    def extra_orient_dim(self):
        """
        :return: 极坐标朝向向量的维度
        """
        return 3*2

    def getExtraOrient(self, angles: dict):
        head = np.array([1, 0, 0])  # 头部向前的箭头
        lHand = np.array([0, 0, 1])  # 左手背向上
        rHand = np.array([0, 0, 1])  # 右手背向上

        # 计算跟随坐标系旋转后的rig向量
        limb_rot = self.nao_limb.getLimbRotation(angles)
        arrow = (head, lHand, rHand)
        names = ('Head', 'LHand', 'RHand')
        # 记录极坐标朝向
        orient = []
        for i, name in enumerate(names):
            v0 = arrow[i]
            R = limb_rot[name]
            v1 = np.dot(R, v0)  # 向量旋转
            phi, theta = cartesian_to_polar(*v1)
            orient += [phi, theta]

        return orient
