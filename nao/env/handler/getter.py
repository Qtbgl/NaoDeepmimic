from nao.env.handler.HandleDrive import HandleDrive
from nao.env.handler.HandleAction import HandleAction
from nao.env.handler.HandleObservation import HandleObservation
from nao.env.handler.HandleReward import HandleReward


def get_handlers(env, arguments: dict) -> tuple[HandleDrive, HandleObservation, HandleAction, HandleReward]:
    task = arguments['task']
    if task == '动作模仿-强化学习奖励模仿':
        from NAO_RL.Handler.Task3.HandleAction301 import HandleAction301
        from NAO_RL.Handler.Task3.HandleObservation401 import HandleObservation401
        from NAO_RL.Handler.Task5.HandleDummy105 import HandleDummy105
        from NAO_RL.Driver.Handler.HandleDrive1041 import HandleDrive1041
        drive = HandleDrive1041(env, motion_file)
        obs4dummy = HandleObservation401(
            drive.drivePose, drive.driveLimbVel2, drive.driveImu, drive.driveFeet, drive.driveTorso)
        act4dummy = HandleAction301(drive.base)
        dummy = HandleDummy105(act4dummy, obs4dummy, env.env_access, drive, drive.driveFallen, drive.drivePose)
        drive = drive
        obs = dummy
        act = dummy
        rew = dummy

    return drive, obs, act, rew
