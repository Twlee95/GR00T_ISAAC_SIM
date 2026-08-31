"""Run the existing SONIC evaluator with live PICO SMPL input.

This is an additive entry point: it does not modify eval_agent_trl.py,
train_agent_trl.py, commands.py, or live_smpl_source.py.
"""

from __future__ import annotations

import argparse
import runpy
import sys
from pathlib import Path

import numpy as np
import torch
import zmq


def _parse_live_args() -> argparse.Namespace:
    """Read this wrapper's options and leave the remaining args for eval."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--live-zmq-host", default="localhost")
    parser.add_argument("--live-zmq-port", type=int, default=5556)
    parser.add_argument("--live-zmq-topic", default="pose")
    parser.add_argument("--xr", action="store_true", default=False)
    parser.add_argument("--state-pub-port", type=int, default=5557)
    live_args, remaining = parser.parse_known_args()
    if live_args.xr:
        remaining = ["--xr", *remaining]
    sys.argv = [sys.argv[0], *remaining]
    return live_args


LIVE_ARGS = _parse_live_args()

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gear_sonic import train_agent_trl  # noqa: E402
from gear_sonic.teleop_realtime.live_smpl_source import (  # noqa: E402
    LiveSmplBuffer,
    LiveSmplMotionLib,
)


class LatestFrameSmplMotionLib(LiveSmplMotionLib):
    """Live source variant that handles batched body_quat_w correctly."""

    def latest_root_quat(self):
        real = object.__getattribute__(self, "_real")
        data = self._latest()
        if data is None or "body_quat_w" not in data:
            return None

        body_quat = np.asarray(data["body_quat_w"])
        if body_quat.ndim == 1:       # (4,)
            root_quat = body_quat
        elif body_quat.ndim == 2:     # (frames, 4)
            root_quat = body_quat[-1]
        elif body_quat.ndim == 3:     # (frames, joints, 4)
            root_quat = body_quat[-1, 0]
        else:
            raise ValueError(f"Unexpected body_quat_w shape: {body_quat.shape}")

        if root_quat.shape != (4,):
            raise ValueError(f"Unexpected root quaternion shape: {root_quat.shape}")

        return torch.as_tensor(root_quat, dtype=torch.float32, device=real._device)


_original_create_manager_env = train_agent_trl.create_manager_env


def _create_live_manager_env(config, device, args_cli):
    """Create the normal environment, then attach the live PICO source."""
    import isaaclab.utils.noise as _noise
    _noise.__dict__["AdditiveUniformNoiseCfg"] = _noise.UniformNoiseCfg
    _noise.__dict__["AdditiveGaussianNoiseCfg"] = _noise.GaussianNoiseCfg
    print("[compat] noise alias injected", flush=True)
    from isaaclab.sim import SimulationCfg as _SimCfg
    if not hasattr(_SimCfg, "physx"):
        class _PhysxSink:
            def __setattr__(self, k, v): pass
            def __getattr__(self, k): return None
        _SimCfg.physx = property(lambda self: self.__dict__.setdefault("_physx_sink", _PhysxSink()))
        print("[compat] SimulationCfg.physx sink installed", flush=True)
    try:
        _ev = config.manager_env.events
        for _k in ("base_com", "randomize_rigid_body_mass", "physics_material", "add_joint_default_pos", "push_robot"):
            if _k in _ev:
                _ev[_k] = None
        print("[compat] startup DR events disabled", flush=True)
    except Exception as _e:
        print("[compat] events disable skipped:", _e, flush=True)
    env = _original_create_manager_env(config, device, args_cli)

    state_context = zmq.Context.instance()
    state_socket = state_context.socket(zmq.PUB)
    state_socket.setsockopt(zmq.SNDHWM, 1)
    state_socket.bind(f"tcp://*:{LIVE_ARGS.state_pub_port}")

    state_robot = env.motion_command.robot
    body_joint_names = [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]
    state_name_to_id = {
        name: index
        for index, name in enumerate(state_robot.joint_names)
    }
    state_joint_ids = [
        state_name_to_id[name]
        for name in body_joint_names
    ]

    env._sonic_state_socket = state_socket
    env._sonic_state_joint_ids = state_joint_ids

    print(
        f"[SONIC-STATE] publishing tcp://*:{LIVE_ARGS.state_pub_port}",
        flush=True,
    )
    motion_command = env.motion_command
    if motion_command is None:
        raise RuntimeError("The environment has no 'motion' command")

    live_buffer = LiveSmplBuffer(
        zmq_host=LIVE_ARGS.live_zmq_host,
        zmq_port=LIVE_ARGS.live_zmq_port,
        topic=LIVE_ARGS.live_zmq_topic,
    )
    live_motion_lib = LatestFrameSmplMotionLib(
        motion_command.motion_lib,
        live_buffer,
    )

    # ManagerEnvWrapper caches the same library in two places.
    motion_command.motion_lib = live_motion_lib
    env._motion_lib = live_motion_lib

    # Keep strong references for the full evaluator lifetime.
    env._live_smpl_buffer = live_buffer
    env._live_smpl_motion_lib = live_motion_lib

    command_cls = type(motion_command)
    if not hasattr(command_cls, "_offline_smpl_root_quat_w"):
        offline_property = command_cls.smpl_root_quat_w
        command_cls._offline_smpl_root_quat_w = offline_property

        def live_root_quat_w(self):
            latest = getattr(self.motion_lib, "latest_root_quat", lambda: None)()
            if latest is None:
                return self.__class__._offline_smpl_root_quat_w.fget(self)
            return latest.reshape(1, 4).expand(self.num_envs, 4).contiguous()

        command_cls.smpl_root_quat_w = property(live_root_quat_w)

    print(
        "[LivePICO] connected to "
        f"tcp://{LIVE_ARGS.live_zmq_host}:{LIVE_ARGS.live_zmq_port} "
        f"topic={LIVE_ARGS.live_zmq_topic!r}",
        flush=True,
    )
    _orig_step = env.step
    _step_n = {"i": 0}
    def _logged_step(*a, **k):
        r = _orig_step(*a, **k)

        def tensor_value(value):
            return value.torch if hasattr(value, "torch") else value

        root_state = tensor_value(
            state_robot.data.root_state_w
        )[0].detach().cpu().numpy()
        joint_pos = tensor_value(
            state_robot.data.joint_pos
        )[0, state_joint_ids].detach().cpu().numpy()
        joint_vel = tensor_value(
            state_robot.data.joint_vel
        )[0, state_joint_ids].detach().cpu().numpy()

        try:
            state_socket.send_pyobj(
                {
                    "root_state": root_state,
                    "joint_pos": joint_pos,
                    "joint_vel": joint_vel,
                },
                flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            pass

        _step_n["i"] += 1
        if _step_n["i"] % 50 == 0:
            print(f"[step] {_step_n['i']}", flush=True)
        return r
    env.step = _logged_step
    print("[compat] step logger installed", flush=True)
    return env


# eval_agent_trl imports this same module object and therefore uses the wrapper.
train_agent_trl.create_manager_env = _create_live_manager_env


if __name__ == "__main__":
    runpy.run_path(
        "/workspace/wbc/gear_sonic/sonic_xr_v2/sonic_policy_source.py",
        run_name=chr(95) * 2 + "main" + chr(95) * 2,
    )
