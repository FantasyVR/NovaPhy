"""Demo: PPO control of an inverted pendulum articulated body.

===========================================================================
功能简介
===========================================================================
本脚本基于 NovaPhy 物理引擎的关节体动力学 (Articulation + ArticulatedSolver),
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
依赖 (在 conda novaphy 环境下测试通过)
===========================================================================
  Python     >= 3.11
  novaphy    >= 0.1.0   (本地 pip install -e . 安装)
  numpy      >= 2.4
  torch      >= 2.10    (CPU 或 CUDA 均可)
  polyscope  >= 2.6     (仅可视化时需要, pip install polyscope)

===========================================================================
使用方法 (在 NovaPhy 根目录下)
===========================================================================
  # 训练 PPO 模型（权重自动保存到 demos/checkpoints/ 下）
  python demos/demos_ppo_inverted_pendulum.py --train

  # 训练完成后立即可视化
  python demos/demos_ppo_inverted_pendulum.py --train --visual

  # 使用已有权重进行可视化
  python demos/demos_ppo_inverted_pendulum.py --visual --model-path <权重路径>
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
    dir_name = f"{timestamp}_{algorithm}_{task}_{asset}"
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
