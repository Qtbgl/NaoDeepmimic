class EnvAccess:
    """
    - HandleEnv的数据接口
    """

    def __init__(self):
        # 时间记录
        self._count_step = 0  # episode步数
        self._epi_time = 0  # episode时间，单位ms
        self._episode = -1  # 当前的episode，下标从零开始
        self._sim_time = 0  # 持续模拟时间，单位ms

    @property
    def count_step(self):  # 外部访问接口
        return self._count_step

    @property
    def epi_time(self):  # 外部访问接口
        return self._epi_time

    @property
    def episode(self):  # 外部访问接口
        return self._episode

    @property
    def sim_time(self):  # 外部访问接口
        return self._sim_time

    @sim_time.setter
    def sim_time(self, value):  # 保留其他类改写接口
        self._sim_time = value
