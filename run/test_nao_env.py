from nao.env.WebotsNao import WebotsNao
from nao.env.handler.impl.HandleDrive1041 import HandleDrive1041


def checkenv():
    file = r'..\nao\mo\data\Webots\legs\Forwards50.csv'
    env = WebotsNao(task='动作模仿-强化学习奖励模仿', motion_file=file)
    drive = env.env_method
    assert isinstance(drive, HandleDrive1041)

    for i in range(10):  # 评估模型
        observations = env.reset()
        done = False
        cnt = 0
        reward_ep = 0
        while not done:
            action = env.act.getActionInOrder(env.act.drive.sample_angles())
            # print('....................................................')
            observations, reward, done, info = env.step(action)
            print('reward', reward)
            # print('sim_time', env.env_access.sim_time)
            cnt += 1

        print('step cnt', cnt)


if __name__ == '__main__':
    checkenv()
