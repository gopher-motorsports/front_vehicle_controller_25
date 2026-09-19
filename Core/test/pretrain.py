"""
Pretrain a policy by imitating the reference driver (behaviour cloning), as a
starting point for PPO.

1. Record: the reference driver drives staged runs and rolling spawns from
   every curriculum range (for skidpad, late in each of the four laps, so every
   gate decision is shown). A little Gaussian noise is added to the commands it
   executes, so the car also visits slightly off-line states; the label stored
   is always the driver's clean command for that state.
2. Fit: the PPO policy network is trained to output those commands (the mean
   of its action distribution) from the policy's own observations: cones,
   speed, phase flags, next_turn. The value network is trained on the
   discounted returns of the recorded runs, so the first PPO updates don't
   wipe out the cloned policy.
3. Save: <out>.zip with <out>_vecnormalize.pkl (observation statistics from the
   recordings) and <out>_config.json, so evaluate.py, gate_check.py and
   train.py --resume-from all work on it unchanged.

The reference driver sees ground truth; the policy only sees cones. Check the
cloned policy with evaluate.py and gate_check.py before fine-tuning.

Usage:
    python pretrain.py --event skidpad --out models/skidpad_bc
    python evaluate.py --model models/skidpad_bc --out skidpad_bc.png
    python gate_check.py --model models/skidpad_bc
    python train.py --event skidpad --resume-from models/skidpad_bc.zip \\
        --learning-rate 1e-4 --timesteps 2000000 --out models/skidpad_bc_ft
"""
import argparse
import os
import sys
import time
from types import SimpleNamespace

import numpy as np
import torch
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from env import make_env
from reference_driver import ReferenceDriver
from train import build_ppo, vecnormalize_path, config_path, _save_config, _existing_outputs

# Reference driver settings per event, as used for the baselines.
DRIVER_KWARGS = {
    "skidpad": dict(speed=9.5, ay_max=10.5),
    "acceleration": dict(),
    "autocross": dict(ay_max=8.5),
}


def record(event, n_staged, n_rolling, noise, reward_scale, gamma, sensor, seed):
    """Run the reference driver; return observations, clean actions, returns."""
    env = make_env(event, sensor=sensor, action_mode="absolute")
    rng = np.random.default_rng(seed)
    obs_all, act_all, ret_all = [], [], []
    statuses = {}
    n_ranges = len(env.track.curriculum_ranges) or 1
    plan = [("staged", None)] * n_staged + [
        ("rolling", i % n_ranges) for i in range(n_rolling)]
    if not env.track.curriculum_ranges:
        plan = [("staged", None)] * (n_staged + n_rolling)
    driver_kwargs = DRIVER_KWARGS.get(env.rules.name, {})
    for ep, (kind, rng_idx) in enumerate(plan):
        options = {"curriculum": kind == "rolling", "track_seed": int(seed + ep)}
        if rng_idx is not None:
            options["curriculum_range"] = rng_idx
        obs, _ = env.reset(seed=int(seed + ep), options=options)
        driver = ReferenceDriver(env, **driver_kwargs)
        ep_obs, ep_act, ep_rew = [], [], []
        for _ in range(env.max_steps):
            action = np.asarray(driver(obs), dtype=np.float32)
            ep_obs.append(obs)
            ep_act.append(np.clip(action, -1.0, 1.0))
            executed = action + rng.normal(0.0, noise, size=2).astype(np.float32)
            obs, r, term, trunc, info = env.step(np.clip(executed, -1.0, 1.0))
            ep_rew.append(r * reward_scale)
            if term or trunc:
                break
        statuses[info["status"]] = statuses.get(info["status"], 0) + 1
        ret = np.zeros(len(ep_rew), dtype=np.float32)
        acc = 0.0
        for t in range(len(ep_rew) - 1, -1, -1):
            acc = ep_rew[t] + gamma * acc
            ret[t] = acc
        obs_all.append(np.array(ep_obs, dtype=np.float32))
        act_all.append(np.array(ep_act, dtype=np.float32))
        ret_all.append(ret)
    return (np.concatenate(obs_all), np.concatenate(act_all),
            np.concatenate(ret_all), statuses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", default="skidpad",
                    choices=["skidpad", "accel", "acceleration", "autocross"])
    ap.add_argument("--out", default=None, help="Default: models/<event>_bc")
    ap.add_argument("--staged", type=int, default=30, help="Staged demonstration runs")
    ap.add_argument("--rolling", type=int, default=120,
                    help="Rolling-spawn demonstration runs, spread over the curriculum ranges")
    ap.add_argument("--noise", type=float, default=0.1,
                    help="Std of noise added to the executed commands")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--bc-log-std", type=float, default=-1.0,
                    help="Action noise log std to leave on the pretrained policy")
    ap.add_argument("--sensor", default="simple", choices=["simple", "lidar_camera"])
    ap.add_argument("--reward-scale", type=float, default=0.05)
    ap.add_argument("--gamma", type=float, default=0.999)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    if args.event == "accel":
        args.event = "acceleration"
    if args.out is None:
        args.out = f"models/{args.event}_bc"
    args.out = args.out[:-4] if args.out.endswith(".zip") else args.out
    existing = _existing_outputs(args.out)
    if existing and not args.overwrite:
        sys.exit(f"ERROR: {args.out} already has files; choose a new --out or pass --overwrite")
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    t0 = time.time()
    obs, act, ret, statuses = record(args.event, args.staged, args.rolling, args.noise,
                                     args.reward_scale, args.gamma, args.sensor, args.seed)
    print(f"Recorded {len(obs)} samples from {args.staged + args.rolling} runs "
          f"in {time.time() - t0:.0f} s; outcomes {statuses}")

    # Same network and normalisation as train.py.
    train_args = SimpleNamespace(
        event=args.event, sensor=args.sensor, action_mode="absolute",
        reward_scale=args.reward_scale, gamma=args.gamma, learning_rate=3e-4,
        sde=False, log_std_init=0.0, grip_scale=None, tire_npz=None,
        resume_from=None, seed=args.seed, launch_assist_speed=3.0, target_kl=None)
    env_kwargs = dict(event=args.event, sensor=args.sensor, action_mode="absolute")
    vec_env = VecNormalize(DummyVecEnv([lambda: make_env(**env_kwargs)]),
                           norm_obs=True, norm_reward=False, clip_obs=10.0,
                           gamma=args.gamma)
    vec_env.obs_rms.mean = obs.mean(axis=0).astype(np.float64)
    vec_env.obs_rms.var = obs.var(axis=0).astype(np.float64)
    vec_env.obs_rms.count = float(len(obs))
    obs_n = np.clip((obs - vec_env.obs_rms.mean) / np.sqrt(vec_env.obs_rms.var + vec_env.epsilon),
                    -10.0, 10.0).astype(np.float32)

    model = build_ppo(train_args, vec_env)
    policy = model.policy
    policy.set_training_mode(True)
    torch.manual_seed(args.seed)
    opt = torch.optim.Adam(policy.parameters(), lr=args.lr)
    X = torch.as_tensor(obs_n)
    A = torch.as_tensor(act)
    R = torch.as_tensor(ret).unsqueeze(1)
    n = len(X)
    rng = np.random.default_rng(args.seed)
    for epoch in range(args.epochs):
        perm = rng.permutation(n)
        a_loss = v_loss = 0.0
        for i in range(0, n, args.batch_size):
            idx = perm[i:i + args.batch_size]
            x = X[idx]
            dist = policy.get_distribution(x)
            mean = dist.distribution.mean
            loss_a = torch.mean((mean - A[idx]) ** 2)
            loss_v = torch.mean((policy.predict_values(x) - R[idx]) ** 2)
            loss = loss_a + 0.5 * loss_v
            opt.zero_grad()
            loss.backward()
            opt.step()
            a_loss += loss_a.item() * len(idx)
            v_loss += loss_v.item() * len(idx)
        if epoch == 0 or (epoch + 1) % 5 == 0:
            print(f"epoch {epoch + 1:3d}: action MSE {a_loss / n:.4f}, value MSE {v_loss / n:.4f}")
    with torch.no_grad():
        policy.log_std.fill_(args.bc_log_std)
    policy.set_training_mode(False)

    model.save(args.out)
    vec_env.save(vecnormalize_path(args.out))
    _save_config(config_path(args.out), train_args, True,
                 pretrained_from="reference_driver", bc_samples=int(n),
                 bc_staged=args.staged, bc_rolling=args.rolling, bc_noise=args.noise)
    print(f"Saved pretrained model to {args.out}.zip")


if __name__ == "__main__":
    main()
