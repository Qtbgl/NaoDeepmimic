import pandas as pd


class MotionInterp1d:
    _df: pd.DataFrame  # 列有Time+各关节转角
    _names: list[str]

    def getMotorNames(self):
        return self._names

    def getMotionByTime(self, time: int):
        """
        :param time: 单位毫秒
        :return: 1.指定时间的插值动作，假如比所有动作前用第一个后用最后一个;
                 2.time相对开始（即第一个动作）的时间
                 3.time相对结尾（最后一个动作）的时间
        """
        # 获取上一帧时间
        df = self._df
        t = df['Time'] <= time
        # 上一帧的下标，当前从former帧开始，取值[-1, len(t) - 1]
        former = sum(t) - 1
        end = len(t) - 1
        # 边界判断，是否在记录区域之外
        if former == -1:
            motion = {name: df[name][0] for name in self._names}
            # is_end = False
        elif former == end:
            motion = {name: df[name][end] for name in self._names}
            # is_end = True
        else:
            # 在记录区域上或内部
            # 命中时间点判断
            if df['Time'][former] == time:
                motion = {name: df[name][former] for name in self._names}
            else:
                motion = {}
                area = [former, former + 1]
                x1, x2 = df['Time'][area]
                for name in self._names:
                    y1, y2 = df[name][area]
                    slope = (y2 - y1) / (x2 - x1)
                    motion[name] = y1 + slope * (time - x1)  # 目标角度

            # is_end = False

        # 计算相对动作开始、结尾的时间
        start_time = time - df['Time'][0]
        end_time = time - df['Time'][end]
        return motion, start_time, end_time

    def is_end(self, time: int):
        """
        :return: 是否在最后一个动作之外
        """
        # 计算比当前时间小的帧的个数
        df = self._df
        t = df['Time'] < time
        # minor取值[0, len(t)]
        minor = sum(t)
        if minor == len(t):
            return True
        else:
            return False
