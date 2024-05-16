from nao.fk.NaoKeypoint import NaoKeypoint
from nao.fk.NaoLimbRotation import NaoLimbRotation
from controller import Accelerometer, Robot, Gyro, PositionSensor, TouchSensor, Motor, InertialUnit

from nao.fk.NaoSelfPose import NaoSelfPose
from nao.fk.NaoSelfLimbPose import NaoSelfLimbPose
from nao.fk.load_csv import loading


class RobotEquipment:
    def getDevice(self, name):
        return self.robot.getDevice(name)

    def getAccelerometer(self):
        acc = self.getDevice('accelerometer')
        assert isinstance(acc, Accelerometer)
        return acc

    def getGyro(self):
        gyro = self.getDevice('gyro')
        assert isinstance(gyro, Gyro)
        return gyro

    def getInertialUnit(self):
        imu = self.getDevice('imu_my')
        assert isinstance(imu, InertialUnit)
        return imu

    def getMotor(self, name):
        motor = self.getDevice(name)
        assert isinstance(motor, Motor)
        return motor

    def getTouchSensor(self, name):
        sensor = self.getDevice(name)
        assert isinstance(sensor, TouchSensor)
        return sensor

    def __init__(self, robot: Robot):
        self.robot = robot
        self.acc = self.getAccelerometer()
        self.gyro = self.getGyro()
        self.imu = self.getInertialUnit()
        self.motors = {}
        self.ps = {}
        self.fsr = []

        joint_names = [
            'HeadYaw', 'HeadPitch',
            'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
            'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'
        ]

        # 位姿计算工具类
        self.nao_pose = NaoSelfPose(load_data_fun=loading)
        self.limb_pose = NaoSelfLimbPose(self.nao_pose)
        self.nao_keypoint = NaoKeypoint(self.nao_pose)
        self.nao_limb = NaoLimbRotation(self.nao_pose)

        for name in joint_names:
            self.motors[name] = self.getMotor(name)

        for name, motor in self.motors.items():
            self.ps[name] = motor.getPositionSensor()

        for sensor in [self.getTouchSensor(name) for name in ['LFsr', 'RFsr']]:
            self.fsr.append(sensor)
