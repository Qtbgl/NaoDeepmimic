from abc import ABC

from nao.env.EnvAccess import EnvAccess
from nao.env.handler.HandleDummy import HandleDummy
from nao.env.handler.impl.HandleAction301 import HandleAction301


class HandleDummy1(HandleDummy, ABC):
    """
    继承分支使用HandleAction301
    """

    @property
    def action_space(self):
        return self.act.action_space

    def __init__(self, act: HandleAction301, env: EnvAccess):
        super().__init__(act.drive, env)
        # 注入依赖
        self.act = act

    def getActionInOrder(self, motion: dict):
        """
        :param motion: 动作的字典
        :return: 按次序的action列表
        """
        action = [motion.get(name, 0.0) for name in self.act.getMotorNamesInOrder()]
        return action

    def getMotionByOrder(self, action: list):
        """
        :param action: 按次序的action列表
        :return: 动作的字典
        """
        motion = {name: action[i] for i, name in enumerate(self.act.getMotorNamesInOrder())}
        return motion
