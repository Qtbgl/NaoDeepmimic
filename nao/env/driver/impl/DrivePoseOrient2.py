from NAO_MO.mocap.LandmarkWrap import LandmarkWrap
from NAO_MO.pose.PosePresent import PosePresent
from NAO_RL.Driver.RobotDriveImpl.DrivePosePresent import DrivePosePresent


class DrivePoseOrient2(DrivePosePresent):

    @property
    def feature_dim(self):
        """
        :return: 肢体 + 其他部位
        """
        return 8*2 + 3*2

    def _process_feature(self, data: LandmarkWrap) -> list:
        pose = PosePresent(data.world_lm)
        orient = pose.limbOrient
        extra = pose.extraOrient
        return orient + extra
