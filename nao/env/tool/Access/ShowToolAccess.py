from nao.env.tool.draw_tools import MyCube, MyLine
from controller import Supervisor


class ShowToolAccess:
    _supervisor: Supervisor
    _cube: MyCube
    _line: MyLine

    def __init__(self, supervisor: Supervisor):
        """
        :param supervisor: getter调用的依赖
        """
        self._supervisor = supervisor
        print('使用3D可视化工具')

    @property
    def supervisor(self):
        try:
            return self._supervisor
        except AttributeError:
            raise Exception('无法创建依赖')

    @property
    def cube(self):
        try:
            return self._cube
        except AttributeError:
            self._cube = MyCube(self.supervisor)
            return self.cube

    @property
    def line(self):
        try:
            return self._line
        except AttributeError:
            self._line = MyLine(self.supervisor)
            return self.line
