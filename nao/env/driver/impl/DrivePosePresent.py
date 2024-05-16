from NAO_MO.mocap.LandmarkWrap import LandmarkWrap


class DrivePosePresent:
    def __init__(self, landmark_data: list[LandmarkWrap]):
        # 解析姿态特征
        feature_data = []
        timestamp = []
        for data in landmark_data:
            time = data.timestamp
            data = self._process_feature(data)
            # 建立数据索引
            feature_data.append(data)
            timestamp.append(time)

        # 检查最后时间
        assert max(timestamp) == timestamp[-1]
        # 保证时间戳从0开始
        timestamp = [time - timestamp[0] for time in timestamp]

        self.feature_data = feature_data
        self.timestamp = timestamp

    @property
    def feature_dim(self):
        raise NotImplementedError

    def _process_feature(self, data: LandmarkWrap) -> list:
        raise NotImplementedError

    def sample_feature(self, now) -> list:
        """
        :return: 姿态特征向量
        """
        timestamp = self.timestamp
        i = 0
        for i, time in enumerate(timestamp):
            if time > now:
                break
        # 等于或小于当前时间
        i -= 1
        assert i >= 0
        print('sample_feature at', timestamp[i], 'for time', time)  # TODO:
        return self.feature_data[i]

    def is_end(self, now):
        return now >= self.timestamp[-1]  # 时间终点在最后
