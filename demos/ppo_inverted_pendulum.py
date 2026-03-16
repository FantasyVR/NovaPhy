"""Demo: PPO control of an inverted pendulum articulated body.

===========================================================================
功能简介
===========================================================================
本脚本基于 NovaPhy 物理引擎的关节体动力学 (Articulation + ArticulatedSolver)，
搭建了一个 **单自由度倒立摆** 强化学习控制 demo。

物理模型
  - 一根长条刚体通过底部的 Revolute 关节（绕 Z 轴旋转）固定在世界原点。
  - 每个 episode 开始时，关节角度 θ 和角速度 θ̇ 在一个温和的随机范围内初始化，
    使摆杆处于接近倒立但有扰动的状态。

强化学习
  - 算法: Proximal Policy Optimization (PPO)，使用 PyTorch 手写实现。
  - 观测: [cos(θ), sin(θ), θ̇]  — 三维连续状态。
  - 动作: 标量关节力矩 τ ∈ [-max_torque, max_torque]。
  - 目标: 学习一个策略，对摆杆施加力矩使其最终稳定在竖直倒立姿态 (θ ≈ π)。
  - 训练过程中自动保存历史最优（最近 20 episode 平均回报最高）的模型权重。
  - 权重默认保存至 demos/checkpoints/PPO_inverted_pendulum_<时间戳>/best.pth。

可视化
  - 使用 Polyscope 3D 渲染摆杆运动，加载训练好的权重后实时展示策略控制效果。
  - 每个随机初始状态至少演化指定帧数，之后切换到新的随机初态继续展示。
  - 界面顶部居中的 ImGui 面板实时显示当前帧数和施加力矩的大小与方向。

===========================================================================
使用方法 (在 NovaPhy 根目录下，需要 torch 和 polyscope)
===========================================================================
  # 训练 PPO 模型（权重自动保存到 demos/checkpoints/ 下）
  python demos/ppo_inverted_pendulum.py --train

  # 训练完成后立即可视化
  python demos/ppo_inverted_pendulum.py --train --visual

  # 使用已有权重进行可视化
  python demos/ppo_inverted_pendulum.py --visual --model-path <权重路径>
===========================================================================
"""

import os
import sys
import argparse
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy

try:
    import torch
    from torch import nn
except ImportError as e:  # pragma: no cover - optional dependency
    torch = None
    nn = None
    _TORCH_IMPORT_ERROR = e
else:
    _TORCH_IMPORT_ERROR = None

try:
    import polyscope as ps

    HAS_POLYSCOPE = True
except ImportError:  # pragma: no cover - optional dependency
    ps = None
    HAS_POLYSCOPE = False

from novaphy.viz import make_box_mesh, quat_to_rotation_matrix


def _require_torch():
    if torch is None:
        raise ImportError(
            "PyTorch is required for this demo. "
            "Install with: pip install torch"
        ) from _TORCH_IMPORT_ERROR


@dataclass
class PendulumConfig:
    length: float = 1.0
    mass: float = 1.0
    gravity: float = 9.81
    dt: float = 1.0 / 240.0
    max_torque: float = 5.0
    episode_steps: int = 400


def build_single_pendulum(cfg: PendulumConfig) -> novaphy.Articulation:
    art = novaphy.Articulation()

    j = novaphy.Joint()
    j.type = novaphy.JointType.Revolute
    j.axis = np.array([0, 0, 1], dtype=np.float32)
    j.parent = -1
    j.parent_to_joint = novaphy.Transform.identity()
    art.joints = [j]

    body = novaphy.RigidBody()
    body.mass = cfg.mass
    body.com = np.array([0, -cfg.length / 2, 0], dtype=np.float32)
    body.inertia = np.eye(3, dtype=np.float32) * cfg.mass * cfg.length * cfg.length / 3.0

    art.bodies = [body]
    art.build_spatial_inertias()
    return art


class InvertedPendulumEnv:
    """Minimal RL environment wrapping NovaPhy articulated pendulum."""

    def __init__(self, cfg: PendulumConfig):
        self.cfg = cfg
        self.art = build_single_pendulum(cfg)
        self.solver = novaphy.ArticulatedSolver()
        self.gravity = np.array([0, -cfg.gravity, 0], dtype=np.float32)
        self._step_count = 0

        self.q = np.zeros(1, dtype=np.float32)
        self.qd = np.zeros(1, dtype=np.float32)

    def reset(self):
        """Reset with mildly random angle and angular velocity in a 2D plane.

        The pendulum is strictly planar (single revolute joint about Z). To
        avoid overly violent initial states and simplify training, we sample
        the joint angle θ in a small band around the upright configuration and
        the angular velocity θ̇ in a modest range.
        """
        # Angle near upright: θ ≈ π ± ~0.4 rad (~±23°)
        theta = np.pi + np.random.uniform(-0.4, 0.4)
        # Angular velocity in a small range
        theta_dot = np.random.uniform(-1.5, 1.5)
        self.q[:] = theta
        self.qd[:] = theta_dot
        self._step_count = 0
        return self._get_state()

    def _get_state(self):
        theta = float(self.q[0])
        theta_dot = float(self.qd[0])
        # Use sin/cos for angle to avoid discontinuities
        return np.array(
            [np.cos(theta), np.sin(theta), theta_dot],
            dtype=np.float32,
        )

    def step(self, action):
        torque = float(np.clip(action, -self.cfg.max_torque, self.cfg.max_torque))
        tau = np.array([torque], dtype=np.float32)

        # Integrate with a few substeps for stability
        substeps = 4
        sub_dt = self.cfg.dt / substeps
        for _ in range(substeps):
            self.q, self.qd = self.solver.step(
                self.art, self.q, self.qd, tau, self.gravity, sub_dt
            )

        self._step_count += 1

        theta = float(self.q[0])
        theta_dot = float(self.qd[0])

        # Upright corresponds to theta ~ pi in this setup
        theta_err = wrap_angle(theta - np.pi)

        # Quadratic cost encouraging upright and low angular velocity
        cost = theta_err * theta_err + 0.1 * theta_dot * theta_dot + 0.001 * (
            torque * torque
        )
        reward = -cost

        done = False
        if abs(theta_err) > np.pi / 2:  # fallen
            done = True
        if self._step_count >= self.cfg.episode_steps:
            done = True

        return self._get_state(), reward, done, {
            "theta": theta,
            "theta_dot": theta_dot,
            "theta_err": theta_err,
        }


def wrap_angle(x: float) -> float:
    return ((x + np.pi) % (2 * np.pi)) - np.pi


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, hidden_dim: int = 64):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )
        # Log standard deviation for Gaussian policy
        self.log_std = nn.Parameter(torch.zeros(1))

    def forward(self, obs):
        raise NotImplementedError

    def act(self, obs):
        mu = self.actor(obs)
        std = self.log_std.exp().expand_as(mu)
        dist = torch.distributions.Normal(mu, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(-1, keepdim=True)
        value = self.critic(obs)
        return action, log_prob, value

    def evaluate_actions(self, obs, actions):
        mu = self.actor(obs)
        std = self.log_std.exp().expand_as(mu)
        dist = torch.distributions.Normal(mu, std)
        log_probs = dist.log_prob(actions).sum(-1, keepdim=True)
        entropy = dist.entropy().sum(-1, keepdim=True)
        values = self.critic(obs)
        return log_probs, entropy, values


@dataclass
class PPOConfig:
    total_steps: int = 2000_000
    rollout_steps: int = 2048
    num_epochs: int = 10
    minibatch_size: int = 256
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    lr: float = 3e-4
    max_grad_norm: float = 0.5
    value_coef: float = 0.5
    entropy_coef: float = 0.01
    device: str = "cpu"


def compute_gae(rewards, values, dones, cfg: PPOConfig):
    """Compute GAE advantages and returns.

    Expects:
        rewards: shape (T,)
        values:  shape (T+1,)
        dones:   shape (T,)
    Returns:
        advantages: shape (T,)
        returns:    shape (T,)
    """
    T = rewards.shape[0]
    advantages = torch.zeros(T, dtype=rewards.dtype, device=rewards.device)
    gae = 0.0
    for t in reversed(range(T)):
        mask = 1.0 - dones[t]
        delta = rewards[t] + cfg.gamma * values[t + 1] * mask - values[t]
        gae = delta + cfg.gamma * cfg.gae_lambda * mask * gae
        advantages[t] = gae
    returns = advantages + values[:-1]
    return advantages, returns


def _make_checkpoint_dir(
    algorithm: str = "PPO",
    task: str = "inverted",
    asset: str = "pendulum",
) -> str:
    """Build and create a structured checkpoint directory under demos/checkpoints/.

    Layout: demos/checkpoints/<algorithm>_<task>_<asset>_<timestamp>/
    Returns the absolute path to the directory.
    """
    from datetime import datetime

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dir_name = f"{algorithm}_{task}_{asset}_{timestamp}"
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ckpt_dir = os.path.join(script_dir, "checkpoints", dir_name)
    os.makedirs(ckpt_dir, exist_ok=True)
    return ckpt_dir


def train_ppo(
    model_path: str | None = None,
    seed: int = 0,
):
    _require_torch()

    # If no explicit path given, create structured checkpoint directory
    if model_path is None:
        ckpt_dir = _make_checkpoint_dir("PPO", "inverted", "pendulum")
        model_path = os.path.join(ckpt_dir, "best.pth")
        print(f"Checkpoint directory: {ckpt_dir}")
    else:
        os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)

    np.random.seed(seed)
    torch.manual_seed(seed)

    env_cfg = PendulumConfig()
    env = InvertedPendulumEnv(env_cfg)

    obs_dim = 3
    ppo_cfg = PPOConfig()
    device = torch.device(ppo_cfg.device)

    policy = ActorCritic(obs_dim).to(device)
    optimizer = torch.optim.Adam(policy.parameters(), lr=ppo_cfg.lr)

    obs = env.reset()
    obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(device)

    total_steps = 0
    episode_rewards = []
    ep_reward = 0.0
    best_mean_reward = -1e9
    best_saved = False

    while total_steps < ppo_cfg.total_steps:
        obs_buf = []
        actions_buf = []
        logprobs_buf = []
        rewards_buf = []
        dones_buf = []
        values_buf = []

        for _ in range(ppo_cfg.rollout_steps):
            with torch.no_grad():
                action, log_prob, value = policy.act(obs_tensor)
            action_np = action.cpu().numpy()[0, 0]

            next_obs, reward, done, _info = env.step(action_np)

            obs_buf.append(obs)
            actions_buf.append([action_np])
            logprobs_buf.append(log_prob.cpu().numpy())
            rewards_buf.append([reward])
            dones_buf.append([float(done)])
            values_buf.append(value.cpu().numpy())

            ep_reward += reward
            total_steps += 1

            if done:
                episode_rewards.append(ep_reward)
                ep_reward = 0.0
                next_obs = env.reset()

            obs = next_obs
            obs_tensor = torch.from_numpy(obs).float().unsqueeze(0).to(device)

        with torch.no_grad():
            _, _, last_value = policy.act(obs_tensor)
        values_buf.append(last_value.cpu().numpy())

        obs_t = torch.from_numpy(np.asarray(obs_buf, dtype=np.float32)).to(device)
        actions_t = torch.from_numpy(np.asarray(actions_buf, dtype=np.float32)).to(
            device
        )

        # Convert buffers to well-shaped tensors:
        #   rewards_t, dones_t: (T,)
        #   values_t: (T+1,)
        #   logprobs_t: (T, 1)
        logprobs_t = torch.from_numpy(
            np.asarray(logprobs_buf, dtype=np.float32).reshape(-1, 1)
        ).to(device)
        rewards_t = torch.from_numpy(
            np.asarray(rewards_buf, dtype=np.float32).reshape(-1)
        ).to(device)
        dones_t = torch.from_numpy(
            np.asarray(dones_buf, dtype=np.float32).reshape(-1)
        ).to(device)
        values_t = torch.from_numpy(
            np.asarray(values_buf, dtype=np.float32).reshape(-1)
        ).to(device)

        # rewards_t: (T,), values_t: (T+1,), dones_t: (T,)
        advantages, returns = compute_gae(rewards_t, values_t, dones_t, ppo_cfg)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        # add feature dimension back: (T, 1)
        advantages = advantages.unsqueeze(-1)
        returns = returns.unsqueeze(-1)

        b_obs = obs_t
        b_actions = actions_t
        b_logprobs = logprobs_t
        b_advantages = advantages
        b_returns = returns

        batch_size = b_obs.shape[0]
        for _ in range(ppo_cfg.num_epochs):
            idxs = np.random.permutation(batch_size)
            for start in range(0, batch_size, ppo_cfg.minibatch_size):
                end = start + ppo_cfg.minibatch_size
                mb_idx = idxs[start:end]

                mb_obs = b_obs[mb_idx]
                mb_actions = b_actions[mb_idx]
                mb_old_logprobs = b_logprobs[mb_idx]
                mb_adv = b_advantages[mb_idx]
                mb_ret = b_returns[mb_idx]

                new_logprobs, entropy, values = policy.evaluate_actions(
                    mb_obs, mb_actions
                )

                ratio = (new_logprobs - mb_old_logprobs).exp()
                surr1 = ratio * mb_adv
                surr2 = (
                    torch.clamp(ratio, 1.0 - ppo_cfg.clip_range, 1.0 + ppo_cfg.clip_range)
                    * mb_adv
                )
                policy_loss = -torch.min(surr1, surr2).mean()

                value_loss = (mb_ret - values).pow(2).mean()

                entropy_loss = -entropy.mean()

                loss = (
                    policy_loss
                    + ppo_cfg.value_coef * value_loss
                    + ppo_cfg.entropy_coef * entropy_loss
                )

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(policy.parameters(), ppo_cfg.max_grad_norm)
                optimizer.step()

        if episode_rewards:
            recent_mean = float(np.mean(episode_rewards[-20:]))
            print(
                f"Steps: {total_steps:7d} | "
                f"episodes: {len(episode_rewards):4d} | "
                f"avg_reward(last20): {recent_mean:8.3f}"
            )
            if recent_mean > best_mean_reward:
                best_mean_reward = recent_mean
                torch.save(policy.state_dict(), model_path)
                best_saved = True
                print(
                    f"  -> New best mean reward {best_mean_reward:.3f}, "
                    f"saved model to {model_path}"
                )

    # Fallback: if for some reason no best was saved (e.g., no full episodes),
    # save the final weights so there is always a model file.
    if not best_saved:
        torch.save(policy.state_dict(), model_path)
        print(f"Saved final PPO agent to: {model_path}")

    print(f"\nModel weights saved at: {model_path}")
    return model_path


def load_policy(model_path: str, device: str = "cpu") -> ActorCritic:
    _require_torch()
    obs_dim = 3
    policy = ActorCritic(obs_dim)
    policy.load_state_dict(torch.load(model_path, map_location=device))
    policy.to(device)
    policy.eval()
    return policy


def run_visual_agent(model_path: str = "ppo_inverted_pendulum.pth"):
    if not HAS_POLYSCOPE:
        raise ImportError(
            "polyscope is required for visualization. Install with: pip install polyscope"
        )

    policy = load_policy(model_path)
    device = next(policy.parameters()).device

    cfg = PendulumConfig()
    env = InvertedPendulumEnv(cfg)

    ps.init()
    ps.set_program_name("NovaPhy - PPO Inverted Pendulum")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    # Build simple box to represent pendulum link
    half_extents = np.array([0.05, cfg.length / 2, 0.05], dtype=np.float32)
    local_verts, faces = make_box_mesh(half_extents)
    local_verts[:, 1] -= half_extents[1]

    def update_mesh():
        # Always use the latest state from the environment, since env.step()
        # replaces env.q / env.qd with new arrays.
        transforms = novaphy.forward_kinematics(env.art, env.q)
        pos = np.array(transforms[0].position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(transforms[0].rotation, dtype=np.float32))
        world_v = (local_verts @ rot.T) + pos

        if ps.has_surface_mesh("pendulum"):
            ps.get_surface_mesh("pendulum").update_vertex_positions(world_v)
        else:
            m = ps.register_surface_mesh("pendulum", world_v, faces)
            m.set_color((0.2, 0.6, 0.9))

    ps.register_point_cloud("pivot", np.array([[0, 0, 0]], dtype=np.float32))

    obs = env.reset()
    frames_per_seed = 1500
    frame_in_seed = 0
    frame_counter = 0
    last_tau = 0.0

    def callback():
        nonlocal obs, frame_in_seed, frame_counter, last_tau

        for _ in range(4):
            with torch.no_grad():
                obs_t = torch.from_numpy(obs).float().unsqueeze(0).to(device)
                action, _logp, _v = policy.act(obs_t)
            action_np = float(action.cpu().numpy()[0, 0])
            last_tau = action_np
            obs, _reward, _done, _info = env.step(action_np)
            frame_in_seed += 1
            frame_counter += 1
            # 保证每个随机初始状态至少展示 frames_per_seed 帧，然后再随机重置
            if frame_in_seed >= frames_per_seed:
                obs = env.reset()
                frame_in_seed = 0

        # 使用 Polyscope ImGui 展示当前帧数和力矩大小/方向，窗口置于顶部居中
        try:
            import polyscope.imgui as psim  # type: ignore

            viewport_w = ps.get_window_size()[0]
            win_w = 360.0
            psim.SetNextWindowPos((viewport_w / 2.0 - win_w / 2.0, 10.0))
            psim.SetNextWindowSize((win_w, 0.0))
            psim.Begin("PPO Inverted Pendulum Info")
            psim.TextUnformatted(
                f"Frame: {frame_counter} / per-seed {frame_in_seed}/{frames_per_seed}"
            )
            direction = "+Z (CCW)" if last_tau >= 0.0 else "-Z (CW)"
            psim.TextUnformatted(
                f"Torque: {last_tau:.3f} N·m  (dir: {direction})"
            )
            psim.End()
        except Exception:
            pass

        update_mesh()

    ps.set_user_callback(callback)
    ps.show()


def main():
    parser = argparse.ArgumentParser(
        description="PPO control demo for a NovaPhy articulated inverted pendulum."
    )
    parser.add_argument(
        "--train",
        action="store_true",
        help="Run PPO training and save weights.",
    )
    parser.add_argument(
        "--visual",
        action="store_true",
        help="Run visualization using saved PPO weights.",
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="Path to save/load PPO model weights. "
        "When training without this flag, a structured checkpoint directory "
        "is created automatically under demos/checkpoints/.",
    )
    args = parser.parse_args()

    if not args.train and not args.visual:
        parser.print_help()
        return

    saved_path = args.model_path
    if args.train:
        saved_path = train_ppo(model_path=args.model_path)

    if args.visual:
        path = saved_path or args.model_path
        if path is None:
            print(
                "No model path provided. Either train first (--train) or "
                "specify a weights file with --model-path."
            )
            return
        run_visual_agent(model_path=path)


if __name__ == "__main__":
    main()

'''
单关节体倒立摆：从接近倒立出发，绕轴转一整圈（2π）后学习施加力矩使摆杆稳定在倒立姿态。
甩圈阶段用连续角累计；稳定阶段奖励接近 π、角速度小；RL 学习全程力矩策略。

物理验证：可利用 verify_physics_engine() 检验 NovaPhy 在无控摆上的能量守恒与小角度周期，
以体现物理引擎真实性。
'''

import os
import sys
import math
from datetime import datetime
from typing import Tuple, Dict, Any

import numpy as np
import matplotlib.pyplot as plt

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym  # type: ignore
    from gym import spaces  # type: ignore

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

# 保证能找到本地安装的 novaphy（和 demo 脚本一样做法）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import novaphy  # noqa: E402

try:
    import polyscope as ps
    import polyscope.imgui as psim

    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False

from novaphy.viz import make_box_mesh, quat_to_rotation_matrix

# -----------------------------------------------------------------------------
# 单关节旋转倒立：从接近倒立出发，绕轴转一整圈（2π）后学习施加力矩使摆杆稳定在倒立姿态。
# 甩圈阶段用连续角累计；稳定阶段奖励接近 π、角速度小；RL 学习全程力矩策略。
# -----------------------------------------------------------------------------


def make_single_pendulum(length: float = 1.0, mass: float = 1.0) -> novaphy.Articulation:
    """
    单 Revolute 关节摆杆：绕 Z 轴转动，连杆沿 -Y（下垂时 q=0，倒立时 q=π）。
    """
    art = novaphy.Articulation()
    joint = novaphy.Joint()
    joint.type = novaphy.JointType.Revolute
    joint.axis = np.array([0, 0, 1], dtype=np.float32)
    joint.parent = -1
    joint.parent_to_joint = novaphy.Transform.identity()
    art.joints = [joint]
    body = novaphy.RigidBody()
    body.mass = mass
    body.com = np.array([0, -length, 0], dtype=np.float32)
    body.inertia = np.eye(3, dtype=np.float32) * mass * length * length
    art.bodies = [body]
    art.build_spatial_inertias()
    return art


def make_double_pendulum(
    l1: float = 1.0,
    l2: float = 1.0,
    m1: float = 1.0,
    m2: float = 1.0,
) -> novaphy.Articulation:
    """
    与 demos/demo_double_pendulum.py 的 build_double_pendulum 类似：
    - 两个 Revolute 关节构成二连杆
    - 连杆沿 -Y 方向（下垂时 q=[0,0]）
    """
    art = novaphy.Articulation()

    # Joint 0: attached to world origin
    j0 = novaphy.Joint()
    j0.type = novaphy.JointType.Revolute
    j0.axis = np.array([0, 0, 1], dtype=np.float32)
    j0.parent = -1
    j0.parent_to_joint = novaphy.Transform.identity()

    # Joint 1: attached to end of link 0
    j1 = novaphy.Joint()
    j1.type = novaphy.JointType.Revolute
    j1.axis = np.array([0, 0, 1], dtype=np.float32)
    j1.parent = 0
    j1.parent_to_joint = novaphy.Transform.from_translation(np.array([0, -l1, 0], dtype=np.float32))

    art.joints = [j0, j1]

    # Link bodies (thin rods approximation)
    b0 = novaphy.RigidBody()
    b0.mass = m1
    b0.com = np.array([0, -l1 / 2.0, 0], dtype=np.float32)
    b0.inertia = np.eye(3, dtype=np.float32) * m1 * l1 * l1 / 3.0

    b1 = novaphy.RigidBody()
    b1.mass = m2
    b1.com = np.array([0, -l2 / 2.0, 0], dtype=np.float32)
    b1.inertia = np.eye(3, dtype=np.float32) * m2 * l2 * l2 / 3.0

    art.bodies = [b0, b1]

    art.build_spatial_inertias()
    return art


def angle_normalize(x: float) -> float:
    """把角度规范到 [-pi, pi] 区间。"""
    return ((x + math.pi) % (2.0 * math.pi)) - math.pi


class InvertedPendulumEnv(gym.Env):
    """
    单关节旋转倒立摆：
    - mode=\"balance\"：从接近倒立 (θ≈π) 的小扰动开始，只学习稳定倒立；
    - mode=\"spin_and_balance\"：从接近倒立 (θ≈π) 出发、给一定初速度，先绕轴转一整圈（2π）
      再施加力矩稳定在竖直位置；不再随机初始化到任意角度。
    """

    metadata = {"render.modes": ["human"]}

    def __init__(self, mode: str = "spin_and_balance"):
        super().__init__()

        # 兼容非法或带空格的 mode，统一回退到 balance
        if mode not in ("balance", "spin_and_balance"):
            print(f"[InvertedPendulumEnv] Invalid mode '{mode}', fallback to 'balance'")
            mode = "balance"
        self.mode = mode

        self.length = 1.0
        self.mass = 1.0
        self.gravity = np.array([0, -9.81, 0], dtype=np.float32)
        self.dt = 1.0 / 120.0
        self.substeps = 4
        self.max_torque = 6.0
        self.max_speed = 12.0
        self.max_steps_per_episode = 1600

        self.art = make_single_pendulum(self.length, self.mass)
        self.solver = novaphy.ArticulatedSolver()

        # 动作：单个关节力矩
        self.action_space = spaces.Box(
            low=np.array([-self.max_torque], dtype=np.float32),
            high=np.array([self.max_torque], dtype=np.float32),
            shape=(1,),
            dtype=np.float32,
        )

        # 观测：[cos(theta), sin(theta), theta_dot]
        high = np.array([1.0, 1.0, self.max_speed], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        self.q = np.zeros(1, dtype=np.float32)
        self.qd = np.zeros(1, dtype=np.float32)
        self.steps = 0
        self._theta_cont = 0.0
        self._theta_prev = 0.0
        self._theta_start = 0.0
        self._spin_done = False

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self.steps = 0

        if self.mode == "balance":
            # 仅倒立稳定：从接近倒立的小扰动、小角速度开始
            theta = math.pi + 0.2 * np.random.randn()
            theta_dot = 0.5 * np.random.randn()
            self._spin_done = True  # 无需转圈，直接进入稳定阶段
        else:
            # 转一圈再稳定：从接近倒立出发，给中等初速度，策略自行完成甩圈与回收
            theta = math.pi + 0.15 * np.random.randn()
            theta_dot = 3.0 + 0.5 * np.random.randn()
            self._spin_done = False

        self.q = np.array([theta], dtype=np.float32)
        self.qd = np.array([theta_dot], dtype=np.float32)

        # 连续角与起点
        self._theta_prev = float(self.q[0])
        self._theta_cont = float(self.q[0])
        self._theta_start = float(self.q[0])

        obs = self._get_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        self.steps += 1
        tau = np.clip(float(action[0]), -self.max_torque, self.max_torque)
        tau_arr = np.array([tau], dtype=np.float32)

        for _ in range(self.substeps):
            self.qd[0] = float(np.clip(self.qd[0], -self.max_speed, self.max_speed))
            self.q, self.qd = self.solver.step(
                self.art, self.q, self.qd, tau_arr, self.gravity, self.dt
            )
            # 连续角累计，用于判断是否转满一圈（仅 spin_and_balance）
            if self.mode == "spin_and_balance":
                theta = float(self.q[0])
                delta = theta - self._theta_prev
                self._theta_cont += delta
                self._theta_prev = theta

        if self.mode == "spin_and_balance":
            spin_progress = abs(self._theta_cont - self._theta_start) / (2.0 * math.pi)
            if not self._spin_done and spin_progress >= 1.0:
                self._spin_done = True
        else:
            spin_progress = 1.0

        obs = self._get_obs()
        theta = float(self.q[0])
        theta_dot = float(self.qd[0])
        theta_err = angle_normalize(theta - math.pi)

        # 分阶段 torque 惩罚：甩圈阶段轻一点，稳定阶段重一点
        if self.mode == "spin_and_balance" and not self._spin_done:
            torque_pen = 0.001 * tau * tau
        else:
            torque_pen = 0.005 * tau * tau

        if self.mode == "spin_and_balance" and not self._spin_done:
            # 转圈阶段：鼓励完成一圈，避免过大力矩
            reward = 0.03 * abs(theta_dot) - torque_pen + 0.15 * min(spin_progress, 1.0)
        else:
            # 稳定阶段或 balance 模式：倒立稳定，鼓励终止状态完全竖直（θ=π、θ_dot≈0）
            cost = 3.0 * theta_err * theta_err + 0.1 * theta_dot * theta_dot
            reward = -cost - torque_pen
            # 仅在接近竖直且角速度较小时给额外奖励；balance 模式放宽一点以便阶段一有足够奖励信号
            if self.mode == "balance":
                bonus_theta, bonus_dot = 0.08, 0.5  # 约 ±4.6°，便于 balance 预训练收敛
            else:
                bonus_theta, bonus_dot = 0.03, 0.3  # spin_and_balance 稳定阶段：更严
            if abs(theta_err) < bonus_theta and abs(theta_dot) < bonus_dot:
                reward += 2.0

        terminated = False
        truncated = self.steps >= self.max_steps_per_episode
        if (self.mode == "balance" or self._spin_done) and abs(theta_err) > math.pi / 2:
            terminated = True

        info: Dict[str, Any] = {
            "theta": theta,
            "theta_dot": theta_dot,
            "theta_err": theta_err,
            "spin_progress": spin_progress,
            "spin_done": self._spin_done,
        }
        return obs, float(reward), terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        theta = float(self.q[0])
        theta_dot = float(self.qd[0])
        return np.array(
            [
                math.cos(theta),
                math.sin(theta),
                np.clip(theta_dot, -self.max_speed, self.max_speed),
            ],
            dtype=np.float32,
        )

    def render(self, mode: str = "human"):
        print(f"theta={self.q[0]:.3f} qd={self.qd[0]:.3f} spin_done={self._spin_done}")

    def close(self):
        pass


def make_vec_env():
    # 默认用于 spin_and_balance 任务
    return DummyVecEnv([lambda: InvertedPendulumEnv(mode="spin_and_balance")])


def train_ppo(
    balance_timesteps: int = 600_000,
    spin_timesteps: int = 1500_000,
    model_path: str | None = None,
    checkpoint_interval: int = 500_000,
    algorithm: str = "ppo",
    task: str = "inverted",
    asset: str = "pendulum",
) -> str:
    """
    两阶段训练：
    1) balance 阶段：仅学习倒立稳定（mode=\"balance\"）；
    2) spin_and_balance 阶段：在已学到的平衡策略上微调，学习先转一圈再稳定。

    权重保存到 checkpoints/<algorithm>_<task>_<asset>_<start_time>/ 下，并返回该 run 目录路径。
    """
    # 在 checkpoints 下新建本次训练子目录：算法_任务_资产_训练开始时间
    script_dir = os.path.dirname(os.path.abspath(__file__))
    checkpoints_root = os.path.join(script_dir, "checkpoints")
    run_name = f"{algorithm}_{task}_{asset}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    run_dir = os.path.join(checkpoints_root, run_name)
    os.makedirs(run_dir, exist_ok=True)
    print(f"[train_ppo] Run directory: {run_dir}")

    if model_path is None:
        model_path = os.path.join(run_dir, f"{algorithm}_{task}_{asset}")
    else:
        # 若传入的是相对名（无目录），则放到 run_dir 下
        if os.path.dirname(model_path) == "":
            model_path = os.path.join(run_dir, os.path.basename(model_path))

    # 阶段 1：balance 预训练
    env_balance = DummyVecEnv([lambda: InvertedPendulumEnv(mode="balance")])
    model = PPO(
        "MlpPolicy",
        env_balance,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        n_epochs=10,
    )
    print(f"[train_ppo] Balance phase: {balance_timesteps} timesteps")
    model.learn(total_timesteps=balance_timesteps, reset_num_timesteps=False)
    balance_model_path = f"{model_path}_balance"
    model.save(balance_model_path)
    env_balance.close()
    print(f"[train_ppo] Saved balance model to {balance_model_path}")

    # 阶段 2：在 spin_and_balance 任务上微调，并保存多个 checkpoint
    env_spin = DummyVecEnv([lambda: InvertedPendulumEnv(mode="spin_and_balance")])
    model.set_env(env_spin)

    timesteps_done = 0
    checkpoint_idx = 0
    total_timesteps = spin_timesteps
    print(f"[train_ppo] Spin_and_balance phase: {spin_timesteps} timesteps")
    while timesteps_done < total_timesteps:
        remaining = total_timesteps - timesteps_done
        this_chunk = min(checkpoint_interval, remaining)

        model.learn(total_timesteps=this_chunk, reset_num_timesteps=False)
        timesteps_done += this_chunk
        checkpoint_idx += 1

        ckpt_path = f"{model_path}_step_{timesteps_done}"
        model.save(ckpt_path)
        print(f"[train_ppo] Saved checkpoint at {timesteps_done} steps -> {ckpt_path}")

    # 最终模型（用于 spin_and_balance）
    model.save(model_path)
    env_spin.close()
    print(f"Saved PPO model to {model_path}")
    return run_dir


def evaluate_model(
    model_path: str = "ppo_novaphy_inverted_pendulum",
    episodes: int = 5,
    mode: str = "spin_and_balance",
):
    env = InvertedPendulumEnv(mode=mode)
    model = PPO.load(model_path)

    for ep in range(episodes):
        obs, _ = env.reset()
        done = False
        ep_reward = 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            ep_reward += reward
        print(f"Episode {ep}: total reward = {ep_reward:.2f}")

    env.close()


def debug_plot_episode(
    model_path: str = "ppo_novaphy_inverted_pendulum",
    mode: str = "spin_and_balance",
    max_steps: int = 800,
    save_path: str | None = None,
):
    """
    运行单个 episode，记录 theta/theta_dot/torque/reward/spin_progress 时间序列并画图，
    用于诊断策略是否真的“转一圈后回到倒立”。
    """
    env = InvertedPendulumEnv(mode=mode)
    model = PPO.load(model_path)

    obs, _ = env.reset()
    thetas, theta_dots, torques, rewards, spins = [], [], [], [], []

    for _ in range(max_steps):
        action, _ = model.predict(obs, deterministic=True)
        tau = float(action[0])
        obs, reward, terminated, truncated, info = env.step(action)
        thetas.append(info["theta"])
        theta_dots.append(info["theta_dot"])
        torques.append(tau)
        rewards.append(reward)
        spins.append(info.get("spin_progress", 0.0))
        if terminated or truncated:
            break

    env.close()

    t = np.arange(len(thetas)) * env.dt * env.substeps
    fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    axs[0].plot(t, np.unwrap(thetas), label="theta (unwrap)")
    axs[0].axhline(math.pi, color="k", linestyle="--", label="pi")
    axs[0].set_ylabel("theta [rad]")
    axs[0].legend()

    axs[1].plot(t, theta_dots)
    axs[1].set_ylabel("theta_dot [rad/s]")

    axs[2].plot(t, torques)
    axs[2].set_ylabel("torque")

    axs[3].plot(t, spins)
    axs[3].set_ylabel("spin_progress [turns]")
    axs[3].set_xlabel("time [s]")

    plt.tight_layout()
    if save_path is not None:
        plt.savefig(save_path)
        print(f"Saved debug plot to {save_path}")
    else:
        plt.show()


def _run_single_weight_visualization(
    env: "InvertedPendulumEnv",
    model: PPO,
    weight_path: str,
    title: str = "PPO Inverted Pendulum",
    steps_per_frame: int = 2,
):
    """单权重 Polyscope 可视化，不循环、不切换 checkpoint。"""
    ps.init()
    ps.set_program_name(title)
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")
    half = np.array([0.05, env.length / 2.0, 0.05], dtype=np.float32)
    link_verts, link_faces = make_box_mesh(half)
    link_verts[:, 1] -= half[1]
    ps.register_point_cloud("pivot", np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    obs, _ = env.reset()
    frame_counter = 0
    last_tau = 0.0

    def callback():
        nonlocal obs, frame_counter, last_tau
        psim.Begin("Model")
        psim.TextUnformatted(f"Weight: {os.path.basename(weight_path)}")
        psim.TextUnformatted(f"Frame: {frame_counter}")
        tau_dir = "+Z (CCW)" if last_tau >= 0 else "-Z (CW)"
        psim.TextUnformatted(f"Torque: {last_tau:.3f} N·m  (dir: {tau_dir})")
        psim.End()
        for _ in range(steps_per_frame):
            action, _ = model.predict(obs, deterministic=True)
            last_tau = float(action[0])
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                obs, _ = env.reset()
        transforms = novaphy.forward_kinematics(env.art, env.q)
        t0 = transforms[0]
        pos = np.array(t0.position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(t0.rotation, dtype=np.float32))
        world_v = (link_verts @ rot.T) + pos
        name = "pendulum_link"
        if ps.has_surface_mesh(name):
            ps.get_surface_mesh(name).update_vertex_positions(world_v)
        else:
            ps.register_surface_mesh(name, world_v, link_faces)
        frame_counter += 1

    ps.set_user_callback(callback)
    ps.show()


def visualize_training_progress(
    model_path_prefix: str | None = None,
    run_dir: str | None = None,
    weight_path: str | None = None,
    mode: str | None = None,
    frames_per_checkpoint: int = 400,
):
    """
    使用 novaphy.viz 可视化 PPO 策略（倒立摆）。

    若指定 weight_path，则只加载该权重文件进行可视化，不扫描文件夹、不循环切换。
    此时若未传 mode，则根据路径自动选择：*_balance.zip 用 balance，其余用 spin_and_balance。
    若未指定 weight_path，则按 run_dir（或 checkpoints 下最新 run）扫描 *_step_*.zip 并依次循环可视化。
    """
    if not HAS_POLYSCOPE:
        print("Polyscope 未安装，无法进行可视化。请先 `pip install polyscope`。")
        return

    # 指定了 weight_path：只加载该权重，不循环；环境 mode 需与训练一致
    if weight_path is not None:
        path = os.path.abspath(weight_path)
        if not os.path.isfile(path):
            print(f"权重文件不存在: {path}")
            return
        if mode is None:
            mode = "balance" if "_balance.zip" in path and "_step_" not in path else "spin_and_balance"
        elif mode not in ("balance", "spin_and_balance"):
            mode = "spin_and_balance"
        env = InvertedPendulumEnv(mode=mode)
        model = PPO.load(path)
        _run_single_weight_visualization(env, model, path, title="PPO Inverted Pendulum (指定权重)")
        env.close()
        return

    env = InvertedPendulumEnv()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    checkpoints_root = os.path.join(script_dir, "checkpoints")

    if run_dir is not None:
        base_dir = os.path.abspath(run_dir)
        if not os.path.isdir(base_dir):
            print(f"run_dir 不存在: {run_dir}")
            return
        prefix_basename = os.path.basename(model_path_prefix) if model_path_prefix else "ppo_inverted_pendulum"
    else:
        # 未指定 run_dir 时：优先使用 checkpoints 下最新一次训练
        if os.path.isdir(checkpoints_root):
            subdirs = [
                d for d in os.listdir(checkpoints_root)
                if os.path.isdir(os.path.join(checkpoints_root, d))
            ]
            subdirs.sort()
            for candidate in reversed(subdirs):
                cand_path = os.path.join(checkpoints_root, candidate)
                files = os.listdir(cand_path)
                if any(f.endswith(".zip") and "_step_" in f for f in files):
                    base_dir = cand_path
                    run_dir = base_dir
                    prefix_basename = (
                        os.path.basename(model_path_prefix)
                        if model_path_prefix
                        else "ppo_inverted_pendulum"
                    )
                    print(f"[visualize_training_progress] 使用 checkpoints 下最新 run: {run_dir}")
                    break
            else:
                base_dir = None
        else:
            base_dir = None

        if base_dir is None:
            base_dir = os.getcwd()
            prefix_basename = os.path.basename(model_path_prefix) if model_path_prefix else "ppo_novaphy_inverted_pendulum"

    pattern_start = prefix_basename + "_step_"
    all_files = os.listdir(base_dir)
    ckpts = []
    for name in all_files:
        if name.startswith(pattern_start) and name.endswith(".zip"):
            try:
                step_str = name.split("_step_")[-1].split(".")[0]
                step = int(step_str)
                ckpts.append((step, os.path.join(base_dir, name)))
            except ValueError:
                continue

    if not ckpts:
        print("未找到任何 checkpoint 文件，请先运行 train_ppo 进行训练，或使用 weight_path= 指定权重。")
        return

    ckpts.sort(key=lambda x: x[0])
    print("将按以下训练步数可视化策略：", [s for s, _ in ckpts])

    env = InvertedPendulumEnv()

    # 初始化 Polyscope
    ps.init()
    ps.set_program_name("NovaPhy - PPO Inverted Pendulum Training Progress")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    # 构建单连杆 mesh
    half = np.array([0.05, env.length / 2.0, 0.05], dtype=np.float32)
    link_verts, link_faces = make_box_mesh(half)
    link_verts[:, 1] -= half[1]

    ps.register_point_cloud("pivot", np.array([[0.0, 0.0, 0.0]], dtype=np.float32))

    current_ckpt_idx = 0
    model = PPO.load(ckpts[current_ckpt_idx][1])
    obs, _ = env.reset()
    frame_in_ckpt = 0
    last_tau = 0.0

    def callback():
        nonlocal current_ckpt_idx, model, obs, frame_in_ckpt, last_tau

        step, path = ckpts[current_ckpt_idx]
        action, _ = model.predict(obs, deterministic=True)
        last_tau = float(action[0])
        tau_dir = "+Z (CCW)" if last_tau >= 0 else "-Z (CW)"

        psim.Begin("Training Progress")
        psim.TextUnformatted(f"Checkpoint step: {step}")
        psim.TextUnformatted(f"Weight file: {os.path.basename(path)}")
        psim.TextUnformatted(f"Frame in ckpt: {frame_in_ckpt}/{frames_per_checkpoint}")
        psim.TextUnformatted(f"Spin done: {env._spin_done}")
        psim.TextUnformatted(f"Spin progress: {abs(env._theta_cont - env._theta_start) / (2.0 * math.pi):.2f} turns")
        psim.TextUnformatted(f"Torque (force): {last_tau:.3f} N·m  (dir: {tau_dir})")
        psim.End()

        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            obs, _ = env.reset()

        transforms = novaphy.forward_kinematics(env.art, env.q)
        t0 = transforms[0]
        pos = np.array(t0.position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(t0.rotation, dtype=np.float32))
        world_v = (link_verts @ rot.T) + pos

        name = "pendulum_link_training"
        if ps.has_surface_mesh(name):
            ps.get_surface_mesh(name).update_vertex_positions(world_v)
        else:
            ps.register_surface_mesh(name, world_v, link_faces)

        frame_in_ckpt += 1
        if frame_in_ckpt >= frames_per_checkpoint:
            # 切换到下一个 checkpoint
            frame_in_ckpt = 0
            current_ckpt_idx += 1
            if current_ckpt_idx >= len(ckpts):
                # 到最后一个后保持不再切换
                current_ckpt_idx = len(ckpts) - 1
            model = PPO.load(ckpts[current_ckpt_idx][1])
            obs, _ = env.reset()
            print(f"切换到 checkpoint: step = {ckpts[current_ckpt_idx][0]}")

    ps.set_user_callback(callback)
    ps.show()
    env.close()


def visualize_trained_model(
    model_path: str = "ppo_novaphy_inverted_pendulum",
    mode: str | None = None,
    max_frames: int = 2_000,
):
    """
    使用 novaphy.viz + polyscope 可视化训练好的倒立摆策略。
    若未传 mode，则根据路径自动选择：*_balance.zip 用 balance，其余用 spin_and_balance。
    """
    if not HAS_POLYSCOPE:
        print("Polyscope 未安装，无法进行可视化。请先 `pip install polyscope`。")
        return
    if mode is None:
        mode = "balance" if "_balance.zip" in model_path and "_step_" not in model_path else "spin_and_balance"
    elif mode not in ("balance", "spin_and_balance"):
        mode = "spin_and_balance"
    env = InvertedPendulumEnv(mode=mode)
    model = PPO.load(model_path)

    # 初始化 Polyscope
    ps.init()
    ps.set_program_name("NovaPhy - PPO Inverted Pendulum")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    # 构建单连杆 mesh
    half = np.array([0.05, env.length / 2.0, 0.05], dtype=np.float32)
    link_verts, link_faces = make_box_mesh(half)
    link_verts[:, 1] -= half[1]

    obs, _ = env.reset()
    ps.register_point_cloud("pivot", np.array([[0.0, 0.0, 0.0]], dtype=np.float32))

    frame_counter = 0
    last_tau = 0.0

    def callback():
        nonlocal obs, frame_counter, last_tau
        if frame_counter >= max_frames:
            return

        psim.Begin("Model")
        psim.TextUnformatted(f"Weight file: {os.path.basename(model_path)}")
        psim.TextUnformatted(f"Frame: {frame_counter}/{max_frames}")
        tau = last_tau
        tau_dir = "+Z (CCW)" if tau >= 0 else "-Z (CW)"
        psim.TextUnformatted(f"Torque (force): {tau:.3f} N·m  (dir: {tau_dir})")
        psim.End()

        for _ in range(2):
            action, _ = model.predict(obs, deterministic=True)
            last_tau = float(action[0])
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                obs, _ = env.reset()

        transforms = novaphy.forward_kinematics(env.art, env.q)
        t0 = transforms[0]
        pos = np.array(t0.position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(t0.rotation, dtype=np.float32))
        world_v = (link_verts @ rot.T) + pos

        name = "pendulum_link"
        if ps.has_surface_mesh(name):
            ps.get_surface_mesh(name).update_vertex_positions(world_v)
        else:
            ps.register_surface_mesh(name, world_v, link_faces)

        frame_counter += 1

    ps.set_user_callback(callback)
    ps.show()



if False:  # Legacy SB3/Gym entry point disabled; use the CLI `main()` above instead.
    # 0. 物理引擎验证（无控单摆能量守恒 + 小角度周期）
    # verify_physics_engine(theta0=math.pi / 2, theta_dot0=0.0)   # 能量
    # verify_physics_engine(theta0=0.2, theta_dot0=0.0)          # 周期（小角度）

    # 1. 训练（权重保存到 checkpoints/ppo_inverted_pendulum_<时间>/，并返回 run 目录）
    # run_dir = train_ppo()
    # print("Run dir:", run_dir)

    visualize_training_progress(
        weight_path="/home/nova/BUAA/NovaPhy/ppo_inverted_pendulum.pth"
    )