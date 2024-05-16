from nao.env.driver.RobotDrive import RobotDrive


class HandleAction:
    def __init__(self, drive: RobotDrive):
        self.drive = drive

    # 约定动作空间
    @property
    def action_space(self):
        raise NotImplementedError

    def apply_action(self, action):
        raise NotImplementedError
