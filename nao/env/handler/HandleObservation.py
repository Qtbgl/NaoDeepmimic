import math


class HandleObservation:

    # 传感器参数
    acc_max = 2 * 9.7  # m/s²
    gyro_max = 500 / 180 * math.pi  # rad/s
    each_touch_max = 25  # newtons
    position_max = 1000  # millimeter

    # # 归一化后取值
    # torso_state_min = [-1, -1, -1, -1, -1, -math.pi, -math.pi]
    # torso_state_max = [+1, +1, +1, +1, +1, +math.pi, +math.pi]

    # 约定观察空间
    @property
    def observation_space(self):
        raise NotImplementedError

    def get_observations(self):
        raise NotImplementedError

    def get_default_observation(self):
        raise NotImplementedError

    def get_info(self):
        return {}  # 用字典返回
