from collections import OrderedDict


class KeyQueue:
    def __init__(self, max_size):
        self.max_size = max_size
        self._queue = OrderedDict()

    def set(self, key, value):
        if key in self._queue:
            raise ValueError(f"Key '{key}' already exists in the queue.")
        self._queue[key] = value
        # 维护长度，移除较早数据
        if len(self._queue) > self.max_size:
            self._queue.popitem(last=False)

    def get(self, key, default_value=None):
        if key in self._queue:
            return self._queue[key]

        if default_value is None:
            raise KeyError(f"Key '{key}' not found in the queue.")
        else:
            return default_value

    def clear(self):
        self._queue.clear()

    def keys(self):
        return list(self._queue.keys())


def test():
    # 示例用法
    queue = KeyQueue(3)

    queue.set(1, 'value1')
    queue.set(2, 'value2')
    queue.set(3, 'value3')

    # 添加重复的键会引发 ValueError
    queue.set(1, 'value4')  # 抛出 ValueError 异常
    print('keys', queue.keys())

    print(queue.get(2))  # 输出: 'value2'


if __name__ == '__main__':
    test()
