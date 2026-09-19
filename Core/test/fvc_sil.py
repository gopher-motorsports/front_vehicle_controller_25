"""
FVC software-in-the-loop test: the firmware in deploy/firmware (gopher_ds on
top of the controller) drives the simulator, with GopherCAN and Gopher Sense
stubbed. Everything reaches the adapter the way it would on the car:

  - VectorNav values in the network parameters (x forward, y right, z down;
    deg/s; g), wheel speeds in m/s;
  - cone lists encoded with tools/perception_msg.py and fed byte by byte
    through the UART framing;
  - the model loaded from a .fsaepol buffer, as from flash sector 7.

    python deploy/test/fvc_sil.py models/skidpad_pretrained
    python deploy/test/fvc_sil.py models/autocross_combo_v3_best --mode realistic --seeds 5
    python deploy/test/fvc_sil.py models/skidpad_pretrained --seeds 2 --plot results/fvc_sil
    python deploy/test/fvc_sil.py models/skidpad_pretrained --wrong-gyro-sign

Modes:
  exact      true INS velocities and a fresh cone list every step. Observations
             and actions are compared with the trained Python policy; expect
             differences around 1e-3, from the message's 1 cm resolution.
  realistic  INS noise, and cone lists every --frame-every steps (default 3,
             ~17 Hz) delivered --latency-ms late (default 40).
             --estimator wheels uses wheel speeds + IMU instead of the INS
             velocities (vy = 0).

Also checks, once per run: a model for another event is refused, and a
perception dropout faults the adapter.
"""
import argparse
import ctypes as C
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

TEST = os.path.dirname(os.path.abspath(__file__))
DEPLOY = os.path.dirname(TEST)
REPO = os.path.dirname(DEPLOY)
FIRMWARE = os.path.join(DEPLOY, "Src")
sys.path[:0] = [REPO, os.path.join(DEPLOY, "tools"), TEST]

from env import make_env                      # noqa: E402
from train import config_path, vecnormalize_path   # noqa: E402
import export_policy                          # noqa: E402
import fsaepol                                # noqa: E402
import perception_msg                         # noqa: E402

STATES = {0: "idle", 1: "ready", 2: "driving", 3: "finished", 4: "fault"}
FAULTS = {0: "none", 1: "model", 2: "wrong event", 3: "INS stale", 4: "INS invalid",
          5: "wheels stale", 6: "perception stale"}
MAX_WIDTH = 256


class Command(C.Structure):
    _fields_ = [("steer_rad", C.c_float), ("steer_deg", C.c_float),
                ("total_drive_torque_nm", C.c_float), ("brake_front_nm", C.c_float),
                ("brake_rear_nm", C.c_float), ("valid", C.c_uint8)]


class Outputs(C.Structure):          # fsae_outputs_t
    _fields_ = [("action", C.c_float * 2), ("applied", C.c_float * 2),
                ("steer_rad", C.c_float), ("drive_torque_nm", C.c_float),
                ("total_drive_torque_nm", C.c_float),
                ("brake_torque_front_nm", C.c_float), ("brake_torque_rear_nm", C.c_float),
                ("vx_est", C.c_float), ("vy_est", C.c_float), ("n_cones_used", C.c_uint16),
                ("mission_finished", C.c_uint8), ("perception_stale", C.c_uint8),
                ("launch_assist", C.c_uint8), ("obs", C.c_float * MAX_WIDTH)]


CANDIDATE_COMPILERS = ["cc", "gcc", "clang", "x86_64-w64-mingw32-gcc", "cl"]
INSTALL_HINT = """No C compiler found. The test builds deploy/firmware for this machine.
  Windows: install MSYS2 and 'pacman -S mingw-w64-ucrt-x86_64-gcc' (then add its
           bin folder to PATH), or LLVM/clang, or Visual Studio Build Tools and
           run this from a 'Developer Command Prompt' so cl.exe is on PATH.
           STM32CubeIDE's arm-none-eabi-gcc cannot be used: it builds for the
           STM32, not for this machine. LLVM on its own can only build here if
           the Visual Studio Build Tools (the MSVC headers and libraries) are
           also installed; MSYS2's gcc needs nothing else.
  macOS:   install the Xcode command line tools ('xcode-select --install').
  Linux:   install gcc or clang.
Set CC to point at a specific compiler."""


def find_compiler():
    chosen = os.environ.get("CC")
    if chosen:
        if shutil.which(chosen) is None:
            raise SystemExit(f"CC is set to {chosen!r}, which was not found on PATH")
        return chosen
    for name in CANDIDATE_COMPILERS:
        if shutil.which(name):
            return name
    raise SystemExit(INSTALL_HINT)


def compiler_target(cc):
    """The compiler's target triple, e.g. x86_64-pc-windows-msvc; '' if unknown."""
    try:
        return subprocess.run([cc, "-dumpmachine"], capture_output=True, text=True,
                              timeout=20).stdout.strip()
    except Exception:
        return ""


def build():
    lib_name = "fvc_host.dll" if os.name == "nt" else "libfvc_host.so"
    out = os.path.join(DEPLOY, "build", lib_name)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    srcs = [os.path.join(TEST, "stubs.c")] + [os.path.join(FIRMWARE, f) for f in (
        "gopher_ds.c", "fsae_policy.c", "fsae_controller.c", "fsae_msg.c")]
    # Host build for the test only; the strict (-Werror) check is the ARM build
    # of the firmware. Set CC to choose the compiler.
    cc = find_compiler()
    exports = os.path.join(TEST, "host_exports.def")
    if os.path.basename(cc).lower() in ("cl", "cl.exe"):        # MSVC
        objdir = os.path.join(DEPLOY, "build", "obj")
        os.makedirs(objdir, exist_ok=True)
        # MSVC exports nothing from a DLL unless it is listed, and ctypes
        # needs these symbols.
        cmd = [cc, "/nologo", "/O2", "/std:c11", "/LD", f"/I{TEST}", f"/I{FIRMWARE}",
               f"/Fo{objdir}\\", *srcs, f"/Fe:{out}", "/link", f"/DEF:{exports}"]
    else:
        cmd = [cc, "-O2", "-std=gnu11", "-Wall", "-Wextra", "-shared", *srcs,
               f"-I{TEST}", f"-I{FIRMWARE}", "-o", out]
        target = compiler_target(cc)
        if "windows-msvc" in target:
            # clang building for the MSVC runtime: lld-link so Visual Studio's
            # linker is not required, and the export list ctypes needs.
            cmd += ["-fuse-ld=lld", "-Xlinker", f"/DEF:{exports}"]
        elif os.name == "nt":
            cmd += [exports]            # MinGW takes the .def as an input file
        else:
            cmd += ["-fPIC", "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.stderr.strip():
        print(result.stderr.strip())
    if result.returncode != 0:
        raise SystemExit(f"Building the host test library with {cc} failed; see the errors above")
    lib = C.CDLL(out)
    lib.stub_init.restype = C.c_int
    lib.stub_init.argtypes = [C.c_void_p, C.c_uint32, C.c_float, C.c_float, C.c_float,
                              C.c_float, C.c_uint8, C.c_uint8]
    lib.stub_set_time.argtypes = [C.c_uint32]
    lib.stub_set_inputs.argtypes = [C.c_float, C.c_float, C.c_float, C.c_uint16, C.c_float,
                                    C.POINTER(C.c_float), C.c_uint32]
    lib.gds_select_mission.restype = C.c_int
    lib.gds_select_mission.argtypes = [C.c_uint8]
    lib.gds_uart_rx_byte.argtypes = [C.c_uint8]
    lib.gds_command.restype = C.POINTER(Command)
    lib.gds_debug_outputs.restype = C.POINTER(Outputs)
    lib.gds_state.restype = C.c_int
    lib.gds_fault.restype = C.c_int
    return lib


def cones_from_detections(det, h):
    out = []
    for row in det:
        if row[6] > 0.5:
            r, b = row[0] * h["range_scale_m"], row[1] * h["bearing_scale_rad"]
            out.append((r * np.cos(b), r * np.sin(b), int(np.argmax(row[2:6]))))
    return out


class Harness:
    def __init__(self, model_path, estimator="ins", wrong_gyro_sign=False):
        with open(config_path(model_path)) as f:
            self.cfg = json.load(f)
        self.model_path = model_path
        self.lib = build()
        blob = export_policy.export(model_path, os.path.join(tempfile.mkdtemp(), "m.fsaepol"),
                                    parity_runs=0)
        self.h = fsaepol.read(blob)[0]
        self.buf = (C.c_uint8 * len(blob)).from_buffer_copy(blob)
        self.env = make_env(self.cfg["event"], sensor=self.cfg.get("sensor", "simple"),
                            action_mode="absolute", launch_assist_speed=0.0)
        p = self.env.vehicle.p
        # speed from the undriven wheels when not using the INS (front pair for RWD)
        wheel_mask = 0x03 if self.h["driven_wheels"] == 2 else 0x0F
        if self.lib.stub_init(self.buf, len(blob), p.wheel_radius, 0.0,
                              1.0 if wrong_gyro_sign else -1.0, -1.0,
                              1 if estimator == "ins" else 0, wheel_mask) != 0:
            raise SystemExit("gds_init failed: model slot invalid")
        self.event = fsaepol.EVENT_IDS[self.h["event"]]

    def check_mission_guard(self):
        wrong = (self.event + 1) % 3
        ok = self.lib.gds_select_mission(wrong) != 0 and FAULTS[self.lib.gds_fault()] == "wrong event"
        print(f"check: model for {self.h['event']} with mission {fsaepol.EVENT_NAMES[wrong]} -> "
              + ("refused (wrong event)" if ok else "NOT REFUSED"))
        return ok

    def check_dropout(self):
        lib = self.lib
        lib.gds_select_mission(self.event)
        lib.stub_set_time(1000)
        lib.gds_go()
        w = (C.c_float * 4)(0, 0, 0, 0)
        for k in range(20):
            now = 1000 + 20 * k
            lib.stub_set_time(now)
            lib.stub_set_inputs(0, 0, 0, 0, 0, w, now)
            lib.gds_tick()
        ok = STATES[lib.gds_state()] == "fault" and FAULTS[lib.gds_fault()] == "perception stale"
        print("check: no cone lists for 400 ms -> "
              + ("fault (perception stale)" if ok else "NO FAULT"))
        lib.gds_stop()
        return ok

    def run(self, mode, seed, frame_every=3, latency_ms=40, ins_noise=True, rng=None, model=None,
            norm=None):
        lib, env, h = self.lib, self.env, self.h
        p = env.vehicle.p
        if mode == "exact":
            frame_every, latency_ms, ins_noise = 1, 0, False
        rng = rng or np.random.default_rng(seed)
        assert lib.gds_select_mission(self.event) == 0
        obs, _ = env.reset(seed=seed, options={"curriculum": False})
        t0 = 1000                      # the adapter treats a receive time of 0 as never received
        lib.stub_set_time(t0)
        lib.gds_go()
        tr = {k: [] for k in ("t", "vx", "vy", "r", "vx_est", "vy_est", "action", "applied",
                              "ref_action", "frame_age", "cones_used", "assist", "stale",
                              "cte", "reward")}
        tr["arrivals"] = []
        queue, last_capture, fault, info = [], None, None, {}
        total_reward, worst_obs, worst_act = 0.0, 0.0, 0.0
        for k in range(env.max_steps):
            now = t0 + 20 * k
            lib.stub_set_time(now)
            st = env.vehicle.state
            if k % frame_every == 0:
                queue.append((now + latency_ms, now, cones_from_detections(env.last_detections, h),
                              env._next_cross))
            while queue and queue[0][0] <= now:
                _, t_cap, cones, crossings = queue.pop(0)
                for byte in perception_msg.frame(perception_msg.encode(cones, crossings, now - t_cap)):
                    lib.gds_uart_rx_byte(byte)
                last_capture = t_cap
                if mode != "exact":
                    tr["arrivals"].append((now - t0) / 1000.0)
            n = (lambda s: rng.normal(0, s)) if ins_noise else (lambda s: 0.0)
            wheels = (C.c_float * 4)(*[float(w * p.wheel_radius) for w in st[6:10]])
            # VectorNav axes: y right, z down
            lib.stub_set_inputs(float(-np.degrees(st[5]) + n(0.5)), float(st[3] + n(0.05)),
                                float(-st[4] + n(0.05)), 0,
                                float(env.vehicle._prev_ax / 9.80665 + n(0.005)), wheels, now)
            lib.gds_tick()
            state = STATES[lib.gds_state()]
            out = lib.gds_debug_outputs().contents
            a_ref = (np.nan, np.nan)
            if mode == "exact" and state == "driving" and model is not None:
                c_obs = np.array(out.obs[: len(obs)], np.float32)
                worst_obs = max(worst_obs, float(np.max(np.abs(c_obs - obs))))
                a_ref = model.predict(norm(obs), deterministic=True)[0]
                worst_act = max(worst_act, float(np.max(np.abs(np.array(out.action) - a_ref))))
            if state == "fault":
                fault = FAULTS[lib.gds_fault()]
                cmd = np.array([0.0, -1.0], np.float32)       # emergency brake
            elif state == "finished":
                cmd = np.array([0.0, -1.0], np.float32)       # Finished: brakes held
            else:
                c = lib.gds_command().contents
                if c.total_drive_torque_nm > 0:
                    pedal = c.total_drive_torque_nm / (h["driven_wheels"] * h["max_motor_torque"])
                else:
                    pedal = -(c.brake_front_nm + c.brake_rear_nm) / h["max_brake_torque"]
                cmd = np.array([c.steer_rad / h["max_steer_rad"], pedal], np.float32)
            tr["t"].append((now - t0) / 1000.0)
            tr["vx"].append(float(st[3])); tr["vy"].append(float(st[4])); tr["r"].append(float(st[5]))
            tr["vx_est"].append(out.vx_est); tr["vy_est"].append(out.vy_est)
            tr["action"].append(tuple(out.action)); tr["applied"].append(tuple(cmd))
            tr["ref_action"].append(tuple(a_ref))
            tr["frame_age"].append(np.nan if last_capture is None else (now - last_capture) / 1000.0)
            tr["cones_used"].append(out.n_cones_used)
            tr["assist"].append(out.launch_assist); tr["stale"].append(state == "fault")
            obs, reward, term, trunc, info = env.step(cmd)
            total_reward += reward
            tr["cte"].append(float(info["cte"])); tr["reward"].append(reward)
            if term or trunc or (state == "fault" and np.hypot(st[3], st[4]) < 0.1):
                break
        lib.gds_stop()
        for key in list(tr):
            tr[key] = np.array(tr[key])
        tr.update(info=info, traj=np.array(env.trajectory), track=env.track,
                  cone_hit_mask=env._cone_hit.copy(), total_reward=total_reward,
                  seed=seed, mode=mode, fault=fault, adapter_state=STATES[lib.gds_state()],
                  worst_obs=worst_obs, worst_act=worst_act)
        return tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--mode", choices=["exact", "realistic", "both"], default="both")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--frame-every", type=int, default=3,
                    help="realistic: a cone list every N 20 ms steps")
    ap.add_argument("--latency-ms", type=int, default=40,
                    help="realistic: delay from capture to delivery")
    ap.add_argument("--estimator", choices=["ins", "wheels"], default="ins",
                    help="velocities from the VectorNav INS, or wheel speeds + IMU with vy = 0")
    ap.add_argument("--no-ins-noise", action="store_true")
    ap.add_argument("--wrong-gyro-sign", action="store_true",
                    help="Configure the adapter with the wrong yaw-rate sign")
    ap.add_argument("--plot", default=None, metavar="DIR")
    args = ap.parse_args()
    model_path = args.model[:-4] if args.model.endswith(".zip") else args.model

    hz = Harness(model_path, args.estimator, args.wrong_gyro_sign)
    hz.check_mission_guard()
    hz.check_dropout()

    from stable_baselines3 import PPO
    import pickle
    policy = PPO.load(model_path, device="cpu")
    with open(vecnormalize_path(model_path), "rb") as fh:
        vn = pickle.load(fh)
    norm = lambda o: np.clip((o - vn.obs_rms.mean) / np.sqrt(vn.obs_rms.var + vn.epsilon),
                             -vn.clip_obs, vn.clip_obs).astype(np.float32)
    settings = dict(frame_every=args.frame_every, latency_ms=args.latency_ms,
                    estimator=args.estimator)
    if args.plot:
        import plots

    modes = ["exact", "realistic"] if args.mode == "both" else [args.mode]
    by_mode = {}
    for mode in modes:
        traces = []
        for s in range(args.seeds):
            tr = hz.run(mode, s, args.frame_every, args.latency_ms, not args.no_ins_noise,
                        model=policy if mode == "exact" else None, norm=norm)
            traces.append(tr)
            if args.plot:
                plots.plot_run(hz.env, tr, model_path, args.plot, settings)
        by_mode[mode] = traces
        infos = [t["info"] for t in traces]
        done = [i["corrected_time"] for i in infos if i.get("corrected_time") is not None]
        faults = sorted({t["fault"] for t in traces if t["fault"]})
        line = (f"{mode:9s} completed {len(done)}/{len(traces)}"
                + (f", mean corrected {np.mean(done):.3f} s" if done else "")
                + f", cones {np.mean([i.get('cones_hit', 0) for i in infos]):.1f}"
                + f", outcomes {sorted({i['status'] for i in infos})}"
                + (f", adapter faults {faults}" if faults else ""))
        if mode == "exact":
            line += (f"\n          max |obs difference| {max(t['worst_obs'] for t in traces):.1e}, "
                     f"max |action difference| {max(t['worst_act'] for t in traces):.1e}")
        print(line)
    if args.plot and len(by_mode) == 2:
        pairs = {e["seed"]: (e, r) for e, r in zip(by_mode["exact"], by_mode["realistic"])}
        plots.plot_comparison(pairs, model_path, args.plot)
    if args.plot:
        print(f"Plots saved to {args.plot}/")


if __name__ == "__main__":
    main()