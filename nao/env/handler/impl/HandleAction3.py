from abc import ABC

import numpy as np
from gym.spaces import Box

from nao.env.handler.HandleAction import HandleAction


class HandleAction3(HandleAction, ABC):
    def getMotorNamesInOrder(self) -> list[str]:
        """
        :return: 固定次序的电机名字，在已注册电机内
        """
        raise NotImplementedError

    @property
    def action_space(self):
        # 直接驱动电机
        order = self.getMotorNamesInOrder()
        act_low = [self.drive.motors[name].getMinPosition() for name in order]
        act_high = [self.drive.motors[name].getMaxPosition() for name in order]
        return Box(low=np.array(act_low), high=np.array(act_high), dtype=np.float64)

    def apply_action(self, action):
        """
        :param action: 依次为每一个电机的转角，且次序固定
        """
        # 获取特定次序的电机名字
        motor_names = self.getMotorNamesInOrder()
        radians = list(action)
        self.setMotors(motor_names, radians)

    def setMotors(self, motor_names: list, radians: list):
        """
        :param motor_names: 控制电机的名字，需均为已注册
        :param radians: 电机转角，与名字对应
        """
        assert len(motor_names) == len(radians)
        # 设置关节电机转角
        for i, name in enumerate(motor_names):
            motor = self.drive.motors[name]
            # 不对电机转角clip限制
            radian = radians[i]
            motor.setPosition(radian)
