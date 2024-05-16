from nao.env.EnvAccess import EnvAccess


class HandleReward:
    def __init__(self, env: EnvAccess):
        """
        :param env: 用于步数判断
        """
        self.env = env

    def get_reward(self, action):
        """
        :param action: 奖励对应的执行动作
        """
        raise NotImplementedError

    def is_terminated(self):
        """
        :return: 环境自然结束，包括目标成功、失败
        """
        raise NotImplementedError

    def is_truncated(self):
        """
        :return: 人为截断，如到达步数上限
        """
        raise NotImplementedError

    def is_done(self):
        is_terminated = self.is_terminated()
        is_truncated = self.is_truncated()
        # 都运行一遍判断
        return is_truncated or is_terminated
