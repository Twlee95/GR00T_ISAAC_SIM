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


def _parse_live_args() -> argparse.Namespace:
    """Read this wrapper's options and leave the remaining args for eval."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--live-zmq-host", default="localhost")
    parser.add_argument("--live-zmq-port", type=int, default=5556)
    parser.add_argument("--live-zmq-topic", default="pose")
    parser.add_argument("--xr", action="store_true", default=False)
    live_args, remaining = parser.parse_known_args()
    # --xr is consumed here; eval_agent_trl enables AppLauncher XR internally.
    sys.argv = [sys.argv[0], *remaining]
    return live_args


LIVE_ARGS = _parse_live_args()

REPO_ROOT = Path(__file__).resolve().parents[1]
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
    if LIVE_ARGS.xr:
        from isaaclab.ui.xr_widgets import TeleopVisualizationManager, XRVisualization
        XRVisualization.assign_manager(TeleopVisualizationManager)

    env = _original_create_manager_env(config, device, args_cli)

    if LIVE_ARGS.xr:
        from isaaclab_teleop import (
            IsaacTeleopCfg,
            XrAnchorRotationMode,
            XrCfg,
            create_isaac_teleop_device,
        )
        from isaaclab_tasks.manager_based.locomanipulation.pick_place.locomanipulation_g1_env_cfg import (
            _build_g1_locomanipulation_pipeline,
        )

        xr_cfg = XrCfg(
            anchor_pos=(0.0, 0.0, -0.95),
            anchor_rot=(0.0, 0.0, 0.0, 1.0),
        )
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        robot_root = "/World/envs/env_0/Robot"

        pelvis_paths = []
        rigid_pelvis_paths = []
        mesh_count = 0

        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.startswith(robot_root):
                if prim.GetTypeName() == "Mesh":
                    mesh_count += 1
                if "pelvis" in path.lower():
                    pelvis_paths.append(path)
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_pelvis_paths.append(path)

        if rigid_pelvis_paths:
            anchor_path = sorted(rigid_pelvis_paths, key=len)[0]
        elif pelvis_paths:
            anchor_path = sorted(pelvis_paths, key=len)[0]
        else:
            anchor_path = robot_root

        xr_cfg.anchor_prim_path = anchor_path
        xr_cfg.fixed_anchor_height = True

        print(
            f"[SONIC-XR] anchor={anchor_path} "
            f"robot_meshes={mesh_count} pelvis_paths={pelvis_paths[:8]}",
            flush=True,
        )
        xr_cfg.anchor_rotation_mode = XrAnchorRotationMode.FOLLOW_PRIM_SMOOTHED

        teleop_cfg = IsaacTeleopCfg(
            pipeline_builder=_build_g1_locomanipulation_pipeline,
            sim_device=device,
            xr_cfg=xr_cfg,
        )
        xr_device = create_isaac_teleop_device(
            teleop_cfg,
            sim_device=device,
            callbacks={},
            cloudxr_env_file=None,
            auto_launch_cloudxr=False,
        )
        xr_device.__enter__()
        env._sonic_xr_device = xr_device
        print("[SONIC-XR] official anchor session installed", flush=True)
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
        xr_device = getattr(env, "_sonic_xr_device", None)
        if xr_device is not None:
            xr_device.advance()
        r = _orig_step(*a, **k)
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
    runpy.run_module("gear_sonic.eval_agent_trl", run_name="__main__")
