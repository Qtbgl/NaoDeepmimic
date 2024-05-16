import numpy as np
from scipy.spatial.transform import Rotation

from nao.env.driver.RobotDrive import RobotDrive
from nao.env.driver.RobotEquipment import RobotEquipment
from nao.env.tool.space_tools import get_torso_pose_in_ground, get_solid_v_in_ground
from controller import Supervisor


class DriveTorso(RobotDrive):
    def __init__(self, supervisor: Supervisor, equipment: RobotEquipment):
        super().__init__(equipment)
        self.supervisor = supervisor

    def get_torso_height_roll_pitch(self):
        """
        :return: 躯干坐标系相对于大地投影的高度，和x轴、y轴旋转姿态
        """
        t, R = get_torso_pose_in_ground()
        rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
        height = t[2]
        # 舍弃恒为零的yaw值，因为z轴不旋转
        roll, pitch = rpy[:2]
        return height, roll, pitch

    def get_velocity_by_world_in_torso(self):
        """
        :return: 躯干的速度和角速度，相对于世界坐标系，表达在Torso坐标系
        """
        # 获取刚体在世界坐标系下的速度
        velocity = self.supervisor.getFromDef('Torso').getVelocity()
        v_world = np.array(velocity[:3])
        a_world = np.array(velocity[3:])

        # 获取躯干相对世界的姿态
        T = np.array(self.supervisor.getFromDef("Torso").getPose()).reshape((4, 4))
        R = T[:3, :3]

        # 速度向量映射到Torso坐标系下
        v_torso = np.dot(R.T, v_world)
        a_torso = np.dot(R.T, a_world)

        return v_torso, a_torso

    def get_velocity_by_world_in_ground(self):
        """
        :return: 躯干的速度和角速度，相对于世界坐标系，表达在大地坐标系
        """
        v, a = get_solid_v_in_ground('Torso')
        return v, a
