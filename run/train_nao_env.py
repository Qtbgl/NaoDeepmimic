from nao.env.WebotsNao import WebotsNao
from train.train_policy import train_go


def main():
    file = r'..\nao\mo\data\Webots\legs\Forwards50.csv'
    env = WebotsNao(task='动作模仿-强化学习奖励模仿', motion_file=file)
    train_go(env)


if __name__ == '__main__':
    main()
