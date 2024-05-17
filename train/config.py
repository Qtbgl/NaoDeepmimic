from argparse import Namespace
from os import getcwd


class Config(object):
    all_motions = ['backflip', 'cartwheel', 'crawl', 'dance_a', 'dance_b', 'getup_facedown'
                   'getup_faceup', 'jump', 'kick', 'punch', 'roll', 'run', 'spin', 'spinkick',
                   'walk']
    curr_path = getcwd()
    # motion = 'spinkick'
    motion = 'dance_b'  # 具体任务动作
    env_name = "dp_env_v3"

    motion_folder = '/mujoco/motions'
    xml_folder = '/mujoco/humanoid_deepmimic/envs/asset'
    xml_test_folder = '/mujoco_test/'

    mocap_path = "%s%s/humanoid3d_%s.txt"%(curr_path, motion_folder, motion)
    xml_path = "%s%s/%s.xml"%(curr_path, xml_folder, env_name)
    xml_path_test = "%s%s/%s_test.xml"%(curr_path, xml_test_folder, env_name)


args = Namespace(
    checkpoint_dir='checkpoint_tmp', env_id='DeepMimic', g_step=3, load_model_path=None, log_dir='log',
    max_kl=0.01, num_timesteps=1000000.0, policy_entcoeff=0, policy_hidden_size=100,
    pretrained_weight_path=None, save_per_iter=100, save_sample=False, seed=0, stochastic_policy=False,
    task='train', traj_limitation=-1)


def get_task_short_name(args):
    task_name = args.env_id.split("-")[0] + '/'
    task_name += "trpo-"
    task_name += "%s-"%(Config.motion)
    task_name += str(args.seed)
    return task_name
