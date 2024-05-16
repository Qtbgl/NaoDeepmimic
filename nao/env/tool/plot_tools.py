from matplotlib import pyplot as plt


def plot_rewards(episode_rewards: list, ep: int):
    # 绘制当前 episode 的 reward 图像
    plt.plot(range(len(episode_rewards)), episode_rewards, label=f'Episode {ep}')
    plt.xlabel('Steps')
    plt.ylabel('Reward')
    plt.legend()
    plt.show()


def plot_evaluate_rewards(timesteps_rewards: dict):
    """
    Plots the reward trajectory graph for DAgger training.
    :param timesteps_rewards: 字典存储训练的timesteps与episode平均奖励
    """
    timesteps = list(timesteps_rewards.keys())
    rewards = list(timesteps_rewards.values())
    # 绘制训练步数为x轴，评价奖励为y轴
    plt.plot(timesteps, rewards)
    plt.xlabel('Training Timesteps')
    plt.ylabel('Episode Average Rewards')
    plt.title('Reward Rollout during DAgger Training')
    plt.show()
