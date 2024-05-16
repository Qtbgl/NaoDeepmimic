from nao.env.driver.RobotDrive import RobotDrive
from nao.env.handler.impl.HandleAction3 import HandleAction3


class HandleAction301(HandleAction3):
    def __init__(self, drive: RobotDrive):
        super().__init__(drive)
        self.names = [name for name in self.drive.motors]

    def getMotorNamesInOrder(self) -> list[str]:
        """
        :return: 控制全身电机
        """
        return self.names

    def apply_action(self, action):
        """
        :param action: 依次为每一个电机的转角，且次序固定
        """
        super().apply_action(action)
