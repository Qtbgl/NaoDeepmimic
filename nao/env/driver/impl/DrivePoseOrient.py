from NAO_MO.pose.PoseWrap2 import PoseWrap2


class DrivePoseOrient:

    def __init__(self, pose_presents: list[PoseWrap2]):
        self.pose_presents = pose_presents
        self.i = 0  # pose_presents查找下标

    def reset(self):
        self.i = 0

    def sample_orient(self, time):
        presents = self.pose_presents

        # 寻找接近当前时间的记录，默认下标停在上一次查询
        while self.i < len(presents) and presents[self.i].timestamp <= time:
            self.i += 1

        if self.i == 0:
            pass
        elif self.i < len(presents):  # 调整下标为<=time
            self.i -= 1
        else:
            self.i = len(presents) - 1

        print('i', self.i, 'timestamp', presents[self.i].timestamp, 'time', time)
        orient = presents[self.i].orient
        return orient

    def is_end(self):
        return self.i == len(self.pose_presents) - 1  # 下标停止最后
