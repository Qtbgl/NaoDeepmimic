import numpy as np
from scipy.spatial.transform import Rotation

from NAO_RL.Driver.RobotDrive import RobotDrive
from NAO_RL.Driver.RobotDriveImpl.DrivePose import DrivePose
from NAO_RL.Driver.RobotEquipment import RobotEquipment
from NAO_RL.Handler.EnvAccess import EnvAccess
from NAO_RL.Tools.collections.KeyQueue import KeyQueue


class DriveLimbVel2(RobotDrive):
    def __init__(self, equipment: RobotEquipment, env: EnvAccess, drivePose: DrivePose):
        super().__init__(equipment)
        self.env = env
        self.drivePose = drivePose
        self.kqLimbPos = KeyQueue(10)
        self.kqLimbRot = KeyQueue(10)

    def reset(self):
        self.kqLimbPos.clear()  # 不保留上一episode模拟时间的记录
        self.kqLimbRot.clear()

    def update(self):
        sim_ms = self.env.sim_time
        # 获取相对Torso位置和姿态
        joint_angles = self.drivePose.joint_radians
        position = self.nao_keypoint.getKeypoint(joint_angles)
        rotation = self.drivePose.nao_limb.getLimbRotation(joint_angles)
        # 记录位置和姿态
        self.kqLimbPos.set(sim_ms, position)
        self.kqLimbRot.set(sim_ms, rotation)

    @property
    def local_linear_v(self) -> dict[str, np.ndarray]:
        now_ms, last_ms = self._get_now_and_last_time(self.kqLimbPos)
        if now_ms == last_ms:
            now = self.kqLimbPos.get(now_ms)
            last = now
            t = 1
        else:
            now = self.kqLimbPos.get(now_ms)
            last = self.kqLimbPos.get(last_ms)
            t = (now_ms - last_ms) / 1000  # 秒

        # 计算相对Torso速度
        v_torso = {}
        for name in now:
            v_torso[name] = (now[name] - last[name]) / t

        return v_torso

    @property
    def local_angular_v(self):
        now_ms, last_ms = self._get_now_and_last_time(self.kqLimbRot)
        if now_ms == last_ms:
            now = self.kqLimbRot.get(now_ms)
            last = now
            t = 1
        else:
            now = self.kqLimbRot.get(now_ms)
            last = self.kqLimbRot.get(last_ms)
            t = (now_ms - last_ms) / 1000  # 秒

        # 计算相对Torso角速度
        a_torso = {}
        for name in now:
            R1 = now[name]
            R0 = last[name]
            R = np.dot(R1, R0.T)  # 计算last姿态到now的外部旋转
            vec = Rotation.from_matrix(R).as_rotvec()
            a_torso[name] = vec / t  # 轴角除于时间

        return a_torso

    @property
    def local_linear_v_2(self):
        pos_seqs, timestamp = self._get_recent_sequences(self.kqLimbPos, time_during=100)
        assert len(timestamp) >= 2, '记录数量不足'
        linear_v = velocity(pos_seqs, timestamp)
        return linear_v

    @property
    def local_angular_v_2(self):
        rot_seqs, timestamp = self._get_recent_sequences(self.kqLimbRot, time_during=100)
        assert len(timestamp) >= 2, '记录数量不足'
        angular_v = angular_velocity(rot_seqs, timestamp)
        return angular_v

    def _get_now_and_last_time(self, kq: KeyQueue):
        now_ms = self.env.sim_time
        # 查询时间
        times = kq.keys()
        assert now_ms in times, "找不到当前时间记录，无法计算当前速度"
        index = times.index(now_ms)
        if index == 0:
            last_ms = now_ms  # 第一次情况
        else:
            last_ms = times[index - 1]
            assert 0 < now_ms - last_ms < 100, f"当前与上一次时间不合理: now_ms {now_ms} last_ms {last_ms}"

        return now_ms, last_ms

    def _get_recent_sequences(self, kq: KeyQueue, time_during: int):
        """
        :param kq:
        :param time_during:
        :return: 近期筛选出的数据序列字典，对应秒的时间戳
        """
        now_ms = self.env.sim_time
        times = kq.keys()
        assert now_ms in times, "找不到当前时间记录，无法计算当前速度"
        recent = []
        timestamp = []
        # 向前查找时间
        for time_ms in times:
            if now_ms - time_ms <= time_during:
                data = kq.get(time_ms)
                time = time_ms / 1000  # 变为秒
                recent.append(data)
                timestamp.append(time)

        # 转化为键名至时间序列
        names = recent[0].keys()
        seqs = {}
        for name in names:
            seq = []
            for data in recent:
                seq.append(data[name])  # seq应与timestamp同序
            seqs[name] = seq

        return seqs, timestamp


def velocity(seqs: dict[list], timestamp: list[int]):
    """
    :param seqs: 近期数据的序列字典
    :param timestamp: 单位秒
    :return: 键名至速度向量的字典
    """
    x = np.array(timestamp)

    # 计算因变量变化率
    v = {}
    for name in seqs:
        y = np.array(seqs[name])
        # 使用polyfit函数拟合一次多项式
        coefficients = np.polyfit(x, y, 1)
        # 速度即为斜率
        v[name] = coefficients[0, :]

    return v


def angular_velocity(seqs: dict[list], timestamp: list[int]):
    """
    :param seqs:
    :param timestamp: 单位秒
    :return:
    """
    # 计算平均角度变换率
    a = {}
    for name in seqs:
        seq = seqs[name]
        data = []
        for i in range(1, len(seq)):
            t = timestamp[i] - timestamp[i - 1]
            R1 = seq[i]
            R0 = seq[i - 1]
            R = np.dot(R1, R0.T)  # 计算last姿态到now的外部旋转
            vec = Rotation.from_matrix(R).as_rotvec()
            a_vec = vec / t  # 轴角除于时间
            data.append(a_vec)
        data = np.array(data)
        a[name] = data.mean(axis=0)

    return a
