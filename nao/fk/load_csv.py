import numpy as np
import pandas as pd

import os


def loading(joint_names):
    # 读取机器人关节、连杆参数
    file_directory = os.path.dirname(__file__)
    df_joints = pd.read_csv(os.path.join(file_directory, 'nao_joints.csv'))
    df_links = pd.read_csv(os.path.join(file_directory, 'nao_links.csv'))

    # 对应次序载入关节轴
    joint_axis = []
    joint_offsets = []

    for joint_name in joint_names:
        joint_axis_row = df_joints.loc[df_joints['name'] == joint_name, ['Axis-X', 'Axis-Y', 'Axis-Z']].values.flatten()
        joint_offsets_row = df_links.loc[df_links['To'] == joint_name, ['X', 'Y', 'Z']].values.flatten()

        joint_axis.append(joint_axis_row)
        joint_offsets.append(joint_offsets_row)

    joint_axis = np.array(joint_axis, dtype=float)
    joint_offsets = np.array(joint_offsets, dtype=float)

    return joint_offsets, joint_axis
