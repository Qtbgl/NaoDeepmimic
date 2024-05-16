from typing import Generator


class EnvMethod:
    """
    - HandleEnv的回调接口，例如用于控制反转
    """

    def update(self, delta):
        """
        - Webots每次步进时间后回调
        - 例如：用于更新传感器数据
        :param delta: 经过的时间，单位毫秒
        """
        pass

    def reset(self) -> None | Generator:
        """
        - episode开始重置时回调
        :return: 默认None，可返回自定义初始设置生成器
        """
        pass

    def just_continue(self):
        """
        - episode开始回调
        :return: 是否跳过重置，默认否
        """
        pass
