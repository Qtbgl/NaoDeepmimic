from scipy.spatial.transform import Rotation

from nao.env.tool.utilities import to_axis_angle
from controller import Supervisor


class MyCube:
    def __init__(self, supervisor: Supervisor, name='my_cube'):
        """
        :param supervisor: 用于操作世界中的结点
        """
        # 重定向姿态到一个方块
        self.solid = supervisor.getFromDef(name)

    def update(self, rotation: Rotation):
        """
        :param rotation: 更新方块的姿态
        """
        rotation = to_axis_angle(rotation)
        # 重定向到一个方块
        field = self.solid.getField('rotation')
        field.setSFRotation(rotation)


class MyLine:
    def __init__(self, supervisor: Supervisor):
        """
        :param supervisor: 用于操作世界中的结点
        """
        # 动态加载
        from controller.node import getSFNode, getMFNode

        # 重定向向量到一条线
        self.pose = supervisor.getFromDef('my_line')
        shape = getMFNode(self.pose.getField('children'), 0)
        self.indexed_line = getSFNode(shape.getField('geometry'))
        self.coord = getSFNode(self.indexed_line.getField('coord'))

    def update(self, v: list[float], translation: list[float], rotation: Rotation):
        """
        :param v: 展示的向量，长度以米表示
        :param translation: v所处的坐标系相对世界的位移
        :param rotation: v所处的坐标系相对世界的旋转
        """
        rotation = to_axis_angle(rotation)
        # 重定向到一条线
        field = self.pose.getField('rotation')
        field.setSFRotation(rotation)

        field = self.pose.getField('translation')
        field.setSFVec3f(translation)

        field = self.coord.getField('point')
        field.setMFVec3f(1, v)

        field = self.indexed_line.getField('coordIndex')
        field.setMFInt32(0, 0)
        field.setMFInt32(1, 1)

    def update_pose(self, translation: list[float], rotation: Rotation):
        """
        :param translation: 载体相对世界坐标系的位移
        :param rotation: 载体相对世界坐标系的旋转
        """
        rotation = to_axis_angle(rotation)
        self.pose.getField("translation").setSFVec3f(translation)
        self.pose.getField("rotation").setSFRotation(rotation)

    def clear_point(self, size):
        """
        :param size: 初始字段长度 >=2
        """
        # 清空点
        field = self.coord.getField('point')
        # 判断长度小
        while field.getCount() < size:
            field.insertMFVec3f(0, [0, 0, 0])
        # 判断长度多
        while field.getCount() > size:
            field.removeMF(0)
        # 设置默认点
        for i in range(size):
            field.setMFVec3f(i, [0, 0, 0])

    def clear_line(self, size):
        """
        :param size: 初始字段长度 >=3
        """
        # 清空线
        field = self.indexed_line.getField('coordIndex')
        # 判断长度小
        while field.getCount() < size:
            field.insertMFInt32(0, 0)
        # 判断长度多
        while field.getCount() > size:
            field.removeMF(0)
        # 设置默认线
        for i in range(size):
            field.setMFInt32(i, 0)

    def update_point_pairs(self, p1: list, p2: list):
        """
        :param p1: 起点坐标点列表
        :param p2: 终点坐标点列表
        """
        assert len(p1) == len(p2)

        # 点的数量
        number = len(p1)
        self.clear_line(number * 3)  # 起终点和-1
        self.clear_point(number * 2)  # 起终点

        # 描点
        for i, v in enumerate(p1 + p2):  # 先p1上的点后p2的点
            v = list(v)
            self.coord.getField('point').setMFVec3f(i, v)

        # 描线
        for i in range(number):
            begin_index = i  # 起点
            end_index = number + i  # 终点
            field = self.indexed_line.getField('coordIndex')
            field.setMFInt32(i * 3 + 0, begin_index)
            field.setMFInt32(i * 3 + 1, end_index)
            field.setMFInt32(i * 3 + 2, -1)
