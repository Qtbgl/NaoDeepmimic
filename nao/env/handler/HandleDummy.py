from abc import ABC

from nao.env.EnvAccess import EnvAccess
from nao.env.driver.RobotDrive import RobotDrive
from nao.env.handler.HandleAction import HandleAction
from nao.env.handler.HandleObservation import HandleObservation
from nao.env.handler.HandleReward import HandleReward


class HandleDummy(HandleObservation, HandleAction, HandleReward, ABC):
    """
    接口多继承
    """
    def __init__(self, drive: RobotDrive, env: EnvAccess):
        """
        多继承初始化
        """
        HandleObservation.__init__(self)
        HandleAction.__init__(self, drive)
        HandleReward.__init__(self, env)
