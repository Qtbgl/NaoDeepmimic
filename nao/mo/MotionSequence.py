import pandas as pd

from nao.mo.MotionInterp1d import MotionInterp1d


class MotionSequence(MotionInterp1d):
    def __init__(self, motor_names: list):
        self._names = motor_names
        df = pd.DataFrame(columns=motor_names+['Time'])
        self._df = df

    def appendMotion(self, motion: dict, time: int):
        """
        :param motion: 关节与角度的字典
        :param time: 累计时间，需比上一次大，单位毫秒
        """
        df = self._df
        # 获取行数
        row_count = df.shape[0]
        # 检查时间是否递增
        if row_count > 0:
            if not time > df['Time'][row_count - 1]:
                raise Exception('动作序列时间不递增')

        # 增加一行
        for name in self._names:
            df.loc[row_count, name] = motion[name]

        df.loc[row_count, 'Time'] = time
