"""
Train a PPO agent on one of the Driverless dynamic events.

Usage:
    python train.py --event skidpad --timesteps 2000000 --n-envs 8
    python train.py --event autocross --timesteps 10000000   # new course each episode
    python train.py --event accel --out models/accel_ppo
    python train.py --event autocross --sensor lidar_camera   # paper-style perception
    python train.py --event accel --seed 2 --out models/accel_s2
    python train.py --resume-from models/skidpad_ppo_ckpt_500000_steps.zip \
        --out models/skidpad_ppo_more
    python train.py --n-envs 4 --serial      # one process, for debugging

Environments run in parallel subprocesses when --n-envs > 1. That needs the
`if __name__ == "__main__"` guard at the bottom of this file, so keep it.

Files a run writes, all named after --out (say models/accel_ppo):
    models/accel_ppo.zip                       final model
    models/accel_ppo_best.zip                  best model found during training
    models/accel_ppo_ckpt_<steps>_steps.zip    checkpoints
each with matching _vecnormalize.pkl and _config.json files. A run refuses to
start if any of these already exist, unless --overwrite is given, so separate
runs never write over each other.

Best model: every --eval-every steps the current policy drives
--eval-episodes staged runs deterministically (fixed seeds and, for autocross,
fixed courses). The model is saved as <out>_best whenever it completes more of
them, or as many in a lower mean corrected time. PPO can learn to drive and
then lose it; the best model keeps what it found. Evaluate <out>_best, not the
final model.

Runs with the same settings can end very differently. Use --seed, and train
two or three seeds when a result matters.

If tensorboard is installed, logs go to ./tb_logs, including a breakdown of why
episodes are ending, the corrected time of completed runs, and the periodic
evaluation (eval/*).

Observations are normalised with VecNormalize. Rewards are not: they are
multiplied by a fixed --reward-scale instead, so the stall penalty, finish bonus
and the rest keep the proportions reward_check.py verifies. (VecNormalize's
reward normalisation divides by a moving estimate and clips at +-10, which
squashed exactly those terms.) The observation statistics are part of the model: they are saved next to it
(<out>_vecnormalize.pkl, and with every checkpoint) and evaluate.py loads them
automatically. A model is useless without them.

Use --grip-scale (and --tire-npz) with the values from calibrate.py so the
policy trains on a car that matches your measured one. Models trained before
the track and timing changes are not comparable and should be retrained.
"""
import argparse
import glob
import json
import os
import sys

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback, CallbackList

import numpy as np

from env import make_env
from tire import make_tire


class TerminationDiagnosticsCallback(BaseCallback):
    """Log why episodes end, as fractions, every `log_every` steps.

    Categories follow `info["status"]`: completed, dnf_off_course, dnf_lost,
    dnf_spin, dnf_stall, unsafe_stop and timeout. Also logs the mean corrected
    time and cone count of completed staged runs, and the best corrected time.
    """
    CATEGORIES = ("completed", "dnf_off_course", "dnf_lost", "dnf_spin",
                  "dnf_stall", "unsafe_stop", "timeout")

    def __init__(self, log_every=8192, verbose=0):
        super().__init__(verbose)
        self.log_every = log_every
        self.counts = {k: 0 for k in self.CATEGORIES}
        self.adjusted, self.cones = [], []
        self.best_adjusted = None
        self._last_log = 0

    def _on_step(self) -> bool:
        for info, done in zip(self.locals.get("infos", []), self.locals.get("dones", [])):
            if not done:
                continue
            status = info.get("status", "timeout")
            if status in self.counts:
                self.counts[status] += 1
            adj = info.get("corrected_time")
            if adj is not None:
                self.adjusted.append(adj)
                self.cones.append(info.get("cones_hit", 0))
                if self.best_adjusted is None or adj < self.best_adjusted:
                    self.best_adjusted = adj

        if self.num_timesteps - self._last_log >= self.log_every:
            total = sum(self.counts.values())
            if total > 0:
                for k, v in self.counts.items():
                    self.logger.record(f"termination/{k}_frac", v / total)
                self.logger.record("termination/episodes_counted", total)
            if self.adjusted:
                self.logger.record("event/corrected_time_mean", float(np.mean(self.adjusted)))
                self.logger.record("event/cones_mean", float(np.mean(self.cones)))
                self.logger.record("event/timed_runs", len(self.adjusted))
            if self.best_adjusted is not None:
                self.logger.record("event/corrected_time_best", self.best_adjusted)
            self.counts = {k: 0 for k in self.counts}
            self.adjusted, self.cones = [], []
            self._last_log = self.num_timesteps
        return True


def vecnormalize_path(model_path):
    """Where the VecNormalize statistics for a saved model live.

    Final models:  models/x_ppo(.zip)                  -> models/x_ppo_vecnormalize.pkl
    Checkpoints:   models/x_ppo_ckpt_500000_steps.zip  -> models/x_ppo_ckpt_vecnormalize_500000_steps.pkl
    """
    base = model_path[:-4] if model_path.endswith(".zip") else model_path
    head, sep, tail = base.rpartition("_ckpt_")
    if sep and tail.endswith("_steps"):
        return f"{head}_ckpt_vecnormalize_{tail}.pkl"
    return base + "_vecnormalize.pkl"


def config_path(model_path):
    """Where the training settings for a saved model live (see vecnormalize_path)."""
    base = model_path[:-4] if model_path.endswith(".zip") else model_path
    head, sep, tail = base.rpartition("_ckpt_")
    if sep and tail.endswith("_steps"):
        return f"{head}_ckpt_config.json"
    return base + "_config.json"


def _save_config(path, args, normalize, **extra):
    cfg = {
        "event": args.event, "sensor": args.sensor, "action_mode": args.action_mode,
        "normalize": normalize, "reward_scale": args.reward_scale,
        "gamma": args.gamma, "learning_rate": args.learning_rate,
        "sde": args.sde, "log_std_init": args.log_std_init, "grip_scale": args.grip_scale,
        "target_kl": getattr(args, "target_kl", None),
        "tire_npz": args.tire_npz, "resumed_from": args.resume_from,
        "seed": args.seed, "launch_assist_speed": args.launch_assist_speed, **extra,
    }
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)


class BestModelCallback(BaseCallback):
    """Periodically drive staged runs deterministically and keep the best model.

    Score: completion rate first, then mean corrected time of completed runs,
    then (while nothing completes) mean distance covered along the run.
    """

    def __init__(self, args, env_kwargs, normalize, eval_every, n_episodes, verbose=1):
        super().__init__(verbose)
        self.args = args
        self.env_kwargs = env_kwargs
        self.normalize = normalize
        self.eval_every = eval_every
        self.n_episodes = n_episodes
        self.best = None
        self._last = 0

    def _init_callback(self):
        self.eval_env = make_env(**self.env_kwargs, curriculum_prob=0.0)
        self._last = self.num_timesteps

    def _evaluate(self):
        vn = self.model.get_env()
        norm = vn.normalize_obs if isinstance(vn, VecNormalize) else (lambda o: o)
        env = self.eval_env
        completed, times, covered = 0, [], []
        for i in range(self.n_episodes):
            obs, _ = env.reset(seed=10_000 + i,
                               options={"curriculum": False, "track_seed": 10_000 + i})
            info = {}
            for _ in range(env.max_steps):
                action, _ = self.model.predict(norm(obs), deterministic=True)
                obs, _, term, trunc, info = env.step(action)
                if term or trunc:
                    break
            tr = env.track
            span = tr.s[tr.finish_idx] - tr.s[tr.start_idx]
            covered.append(float(np.clip((info.get("s", 0.0) - tr.s[tr.start_idx]) / span, 0, 1)))
            if info.get("corrected_time") is not None:
                completed += 1
                times.append(info["corrected_time"])
        rate = completed / self.n_episodes
        mean_time = float(np.mean(times)) if times else None
        return rate, mean_time, float(np.mean(covered))

    def _on_step(self):
        if self.num_timesteps - self._last < self.eval_every:
            return True
        self._last = self.num_timesteps
        rate, mean_time, covered = self._evaluate()
        self.logger.record("eval/completion", rate)
        self.logger.record("eval/distance_fraction", covered)
        if mean_time is not None:
            self.logger.record("eval/corrected_time_mean", mean_time)
        key = (rate, -mean_time if mean_time is not None else -np.inf, covered)
        if self.best is None or key > self.best:
            self.best = key
            out = self.args.out + "_best"
            self.model.save(out)
            if self.normalize:
                self.model.get_env().save(vecnormalize_path(out))
            _save_config(config_path(out), self.args, self.normalize,
                         best_at_steps=int(self.num_timesteps),
                         eval_completion=rate, eval_corrected_time=mean_time,
                         eval_distance_fraction=covered,
                         eval_episodes=self.n_episodes)
            if self.verbose:
                t = "--" if mean_time is None else f"{mean_time:.3f} s"
                print(f"[best] {self.num_timesteps} steps: completed {rate:.0%}, "
                      f"mean corrected {t}, distance {covered:.0%} -> saved {out}.zip")
        return True


def _existing_outputs(out):
    names = [out + ".zip", out + "_best.zip"]
    found = [p for p in names if os.path.exists(p)]
    found += sorted(glob.glob(glob.escape(out) + "_ckpt_*_steps.zip"))
    return found


def _has_tensorboard():
    try:
        import tensorboard  # noqa: F401
        return True
    except ImportError:
        return False


def build_ppo(args, vec_env):
    """The PPO model used for training. pretrain.py builds the same network."""
    return PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        n_steps=1024,
        batch_size=1024,
        n_epochs=10,
        learning_rate=args.learning_rate,
        gamma=args.gamma,
        gae_lambda=0.95,
        clip_range=0.2,
        target_kl=args.target_kl,
        ent_coef=0.005,  # keeps exploration alive against degenerate optima
        # Optional gSDE with tanh squashing: bounded, smoother exploration.
        # It did not learn faster in testing, so it is off by default.
        use_sde=args.sde,
        sde_sample_freq=4,
        policy_kwargs=dict(net_arch=[128, 128], log_std_init=args.log_std_init,
                           **(dict(squash_output=True) if args.sde else {})),
        # The policy network is small enough that moving batches to a GPU
        # costs more than the forward pass saves.
        device="cpu",
        seed=args.seed,
        tensorboard_log="./tb_logs" if _has_tensorboard() else None,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", type=str, default="skidpad",
                    choices=["skidpad", "accel", "acceleration", "autocross"])
    ap.add_argument("--timesteps", type=int, default=2_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--out", type=str, default=None,
                    help="Model path (default: models/<event>_ppo)")
    ap.add_argument("--checkpoint-every", type=int, default=100_000)
    ap.add_argument("--resume-from", type=str, default=None,
                    help="Path to a saved model (.zip) to continue training from")
    ap.add_argument("--serial", action="store_true",
                    help="Step environments serially in one process. Slower with "
                         "several envs, but easier to debug and profile.")
    ap.add_argument("--sensor", type=str, default="simple",
                    choices=["simple", "lidar_camera"],
                    help="Cone perception model (see sensor_lidar_camera.py)")
    ap.add_argument("--tire-npz", type=str, default=None,
                    help="TTC MF96 coefficients (.npz); default SimpleTire")
    ap.add_argument("--grip-scale", type=float, default=None,
                    help="Tire output derate, as fitted by calibrate.py")
    ap.add_argument("--gamma", type=float, default=0.999,
                    help="Discount; the reward is designed for 0.999")
    ap.add_argument("--learning-rate", type=float, default=3e-4)
    ap.add_argument("--target-kl", type=float, default=None,
                    help="Stop each PPO update early past this KL divergence. Use a small "
                         "value (e.g. 0.003) when fine-tuning a pretrained model")
    ap.add_argument("--reward-scale", type=float, default=0.05,
                    help="Constant multiplier on every reward (default 0.05)")
    ap.add_argument("--log-std-init", type=float, default=0.0,
                    help="Initial log std of the action noise (0 = std 1)")
    ap.add_argument("--sde", action="store_true",
                    help="Squashed gSDE exploration instead of a clipped Gaussian")
    ap.add_argument("--no-normalize", action="store_true",
                    help="Train without VecNormalize")
    ap.add_argument("--action-mode", default="absolute", choices=["target", "delta", "absolute"],
                    help="absolute (default); target and delta are experimental")
    ap.add_argument("--launch-assist-speed", type=float, default=3.0,
                    help="Full throttle from a staged start until this speed (0 disables)")
    ap.add_argument("--seed", type=int, default=None,
                    help="Random seed for the environments and the policy")
    ap.add_argument("--eval-every", type=int, default=100_000,
                    help="Steps between best-model evaluations (0 disables)")
    ap.add_argument("--eval-episodes", type=int, default=5)
    ap.add_argument("--overwrite", action="store_true",
                    help="Allow replacing an existing model and its checkpoints")
    args = ap.parse_args()
    if args.out is None:
        args.out = f"models/{args.event}_ppo"
    args.out = args.out[:-4] if args.out.endswith(".zip") else args.out

    existing = _existing_outputs(args.out)
    if existing and not args.overwrite:
        shown = "\n  ".join(existing[:6]) + ("\n  ..." if len(existing) > 6 else "")
        sys.exit(f"ERROR: {args.out} already has files:\n  {shown}\n"
                 "Choose a new --out, or pass --overwrite to replace them.")
    if existing:
        for p in existing:
            os.remove(p)
        print(f"--overwrite: removed {len(existing)} old model file(s) for {args.out}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    # SubprocVecEnv runs each environment in its own process, so they step in
    # parallel. DummyVecEnv steps them one after another in this process, which
    # is the right choice for a single env or when debugging. Subprocesses cost
    # a little startup time and inter-process traffic per step, so they only pay
    # off above one env.
    use_subproc = args.n_envs > 1 and not args.serial
    env_kwargs = dict(event=args.event, sensor=args.sensor, action_mode=args.action_mode,
                      launch_assist_speed=args.launch_assist_speed,
                      tire=make_tire(args.tire_npz, args.grip_scale))
    vec_env = make_vec_env(
        make_env,
        n_envs=args.n_envs,
        seed=args.seed,
        env_kwargs=dict(env_kwargs, reward_scale=args.reward_scale),
        vec_env_cls=SubprocVecEnv if use_subproc else DummyVecEnv,
    )
    print(f"{args.n_envs} env(s) via {'SubprocVecEnv' if use_subproc else 'DummyVecEnv'}")

    normalize = not args.no_normalize
    if normalize:
        stats = vecnormalize_path(args.resume_from) if args.resume_from else None
        if stats and os.path.exists(stats):
            vec_env = VecNormalize.load(stats, vec_env)
            vec_env.training = True
            # Statistics saved by older runs may have reward normalisation on.
            vec_env.norm_reward = False
            print(f"Loaded normalisation statistics from {stats}")
        else:
            if args.resume_from:
                print(f"WARNING: no normalisation statistics at {stats}; starting fresh")
            vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=False,
                                   clip_obs=10.0, gamma=args.gamma)

    if args.resume_from:
        overrides = {"learning_rate": args.learning_rate}
        if args.target_kl is not None:
            overrides["target_kl"] = args.target_kl
        model = PPO.load(args.resume_from, env=vec_env, custom_objects=overrides)
        model.learning_rate = args.learning_rate
        model._setup_lr_schedule()
        model.target_kl = args.target_kl
        if args.seed is not None:
            model.set_random_seed(args.seed)
        print(f"Resumed from {args.resume_from}, previous total_timesteps={model.num_timesteps}")
    else:
        model = build_ppo(args, vec_env)

    # Checkpoints are named after --out, so every run has its own.
    _save_config(config_path(args.out + "_ckpt_0_steps"), args, normalize)
    ckpt_cb = CheckpointCallback(
        save_freq=max(args.checkpoint_every // args.n_envs, 1),
        save_path=os.path.dirname(args.out) or ".",
        name_prefix=os.path.basename(args.out) + "_ckpt",
        save_vecnormalize=normalize,
    )
    callbacks = [ckpt_cb, TerminationDiagnosticsCallback(log_every=8192)]
    if args.eval_every > 0:
        callbacks.append(BestModelCallback(args, env_kwargs, normalize,
                                           args.eval_every, args.eval_episodes))
    callback = CallbackList(callbacks)

    model.learn(total_timesteps=args.timesteps, callback=callback, progress_bar=True,
                reset_num_timesteps=(args.resume_from is None))
    model.save(args.out)
    _save_config(config_path(args.out), args, normalize)
    print(f"Saved final model to {args.out}.zip (settings in {config_path(args.out)})")
    if normalize:
        vec_env.save(vecnormalize_path(args.out))
        print(f"Saved normalisation statistics to {vecnormalize_path(args.out)}")
    if args.eval_every > 0:
        best = config_path(args.out + "_best")
        if os.path.exists(best):
            with open(best) as f:
                b = json.load(f)
            t = b["eval_corrected_time"]
            print(f"Best model: {args.out}_best.zip from step {b['best_at_steps']}, "
                  f"completed {b['eval_completion']:.0%}"
                  + ("" if t is None else f", mean corrected {t:.3f} s"))


if __name__ == "__main__":
    main()