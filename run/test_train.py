from argparse import Namespace

import gym
from mpi4py import MPI

import logger
import utils.tf_util as U
from debunk_learning import learn
from mlp_policy_trpo import MlpPolicy
from mytest.test_env import CustomEnv
from utils.misc_util import set_global_seeds
from config import Config


def train(env, seed, policy_fn, g_step, policy_entcoeff, pretrained_weight_path,
          num_timesteps, save_per_iter, checkpoint_dir, log_dir, task_name=None):

    workerseed = seed + 10000 * MPI.COMM_WORLD.Get_rank()
    set_global_seeds(workerseed)
    # env.seed(workerseed)  # 自定义环境忽略
    print('进入学习')
    learn(env, policy_fn,
          pretrained_weight_path=pretrained_weight_path,
          g_step=g_step, entcoeff=policy_entcoeff,
          max_timesteps=num_timesteps,
          ckpt_dir=checkpoint_dir, log_dir=log_dir,
          save_per_iter=save_per_iter,
          timesteps_per_batch=256,
          max_kl=0.01, cg_iters=10, cg_damping=0.1,
          gamma=0.995, lam=0.97,
          vf_iters=3, vf_stepsize=1e-3,
          task_name=task_name)


def main():
    args = Namespace(
        checkpoint_dir='checkpoint_tmp', env_id='DeepMimic', g_step=3, load_model_path=None, log_dir='log',
        max_kl=0.01, num_timesteps=1000000.0, policy_entcoeff=0, policy_hidden_size=100,
        pretrained_weight_path=None, save_per_iter=100, save_sample=False, seed=0, stochastic_policy=False,
        task='train', traj_limitation=-1)
    # 进入主函数
    U.make_session(num_cpu=1).__enter__()
    set_global_seeds(args.seed)
    # 暂时的环境
    env = CustomEnv()

    task_name = get_task_short_name(args)

    def policy_fn(name, ob_space, ac_space, reuse=False):
        return MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
                                    reuse=reuse, hid_size=args.policy_hidden_size, num_hid_layers=2)

    if args.task == 'train':
        import logging
        import os.path as osp
        import bench
        if MPI is None or MPI.COMM_WORLD.Get_rank() == 0:
            logger.configure(dir='log_tmp/%s'%task_name)
        if MPI.COMM_WORLD.Get_rank() != 0:
            logger.set_level(logger.DISABLED)
        env = bench.Monitor(env, logger.get_dir() and
                            osp.join(logger.get_dir(), "monitor.json"))
        # env.seed(args.seed)  自定义环境不用随机数种子
        gym.logger.setLevel(logging.WARN)
        task_name = get_task_short_name(args)
        args.checkpoint_dir = osp.join(args.checkpoint_dir, task_name)
        args.log_dir = osp.join(args.log_dir, task_name)

        train(env,
              args.seed,
              policy_fn,
              args.g_step,
              args.policy_entcoeff,
              args.pretrained_weight_path,
              args.num_timesteps,
              args.save_per_iter,
              args.checkpoint_dir,
              args.log_dir,
              task_name)
    else:
        raise NotImplementedError

    env.close()


def get_task_short_name(args):
    task_name = args.env_id.split("-")[0] + '/'
    task_name += "trpo-"
    task_name += "%s-"%(Config.motion)
    task_name += str(args.seed)
    return task_name


if __name__ == '__main__':
    main()
