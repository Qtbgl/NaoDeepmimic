import pandas as pd

from nao.mo.MotionInterp1d import MotionInterp1d


class MotionCSV(MotionInterp1d):
    def __init__(self, path: str):
        # 用csv格式读取
        df = pd.read_csv(path)
        # 先获取电机名字
        self._names = [name for name in df.columns if name in self.joint_names]

        # 换算时间
        if 'Time' not in df.columns:
            time_ms = []
            for t in df['#WEBOTS_MOTION']:
                m, s, ms = [int(s) for s in t.split(':')]
                val = m * 60000 + s * 1000 + ms
                time_ms.append(val)

            # 增加一列记录
            df['Time'] = time_ms  # 以毫秒计

        self._df = df

    joint_names = [
        'HeadYaw', 'HeadPitch',
        'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
        'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
        'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
        'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'
    ]
