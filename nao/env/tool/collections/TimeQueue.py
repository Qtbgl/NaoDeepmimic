from collections import deque


class TimeQueue:
    def __init__(self, max_size):
        self.queue = deque(maxlen=max_size)

    def add_record(self, current_time, data):
        # 检查加入的时间是否递增
        if self.queue and current_time <= self.queue[-1][0]:
            raise ValueError("Invalid time. The current time should be greater than the previous time.")

        record = (current_time, data)
        self.queue.append(record)

    def get_recent_records(self, current_time, time_period):
        recent_records = []
        timestamp = []
        start_time = current_time - time_period
        # 收集近期的记录
        for record in self.queue:
            if record[0] >= start_time:  # 记录的时间戳在起始时间后
                recent_records.append(record[1])
                timestamp.append(record[0])

        return recent_records, timestamp

    def get_recent_one(self, current_time, time_period):
        """
        :return: 在当前时间段内的，否则返回None
        """
        start_time = current_time - time_period
        if self.queue and start_time <= self.queue[-1][0]:
            return self.queue[-1][1]  # 最后一个记录
        else:
            return None

    def clear_records(self):
        self.queue.clear()


def test():
    # 创建一个最大容量为 5 的时间队列
    time_queue = TimeQueue(5)

    # 添加记录
    time_queue.add_record(1.5, "Record 1")
    time_queue.add_record(2.0, "Record 2")
    time_queue.add_record(3.2, "Record 3")
    time_queue.add_record(4.7, "Record 4")
    time_queue.add_record(5.5, "Record 5")

    # 获取最近 2 秒内的记录
    current_time = 6.0
    time_period = 2.0
    recent_records = time_queue.get_recent_records(current_time, time_period)
    print(recent_records)

    # 清空记录
    time_queue.clear_records()


if __name__ == '__main__':
    test()
