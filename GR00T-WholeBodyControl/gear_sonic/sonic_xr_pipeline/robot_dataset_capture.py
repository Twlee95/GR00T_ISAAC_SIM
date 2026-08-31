"""G1 robot-view RGB and joint-state dataset recording."""

import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.recorders.recorders_cfg import (
    ActionStateRecorderManagerCfg,
)
from isaaclab.managers import RecorderTerm, RecorderTermCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass


class RobotDatasetRecorder(RecorderTerm):
    """Record synchronized robot RGB and G1 state after every environment step."""

    _probed = False

    def record_post_step(self):
        if not RobotDatasetRecorder._probed:
            RobotDatasetRecorder._probed = True
            try:
                import omni.usd
                from pxr import UsdGeom, Usd
                st = omni.usd.get_context().get_stage()
                for pa in ['/World/envs/env_0/Kitchen',
                           '/World/envs/env_0/Kitchen/model_table_04',
                           '/World/envs/env_0/Object',
                           '/World/envs/env_0/Robot']:
                    pr = st.GetPrimAtPath(pa)
                    if pr:
                        m = UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        t = m.ExtractTranslation()
                        q = m.ExtractRotationQuat()
                        i = q.GetImaginary()
                        print('[PROBE] %-20s pos=(%.3f, %.3f, %.3f) rot=(%.4f, %.4f, %.4f, %.4f)' % (
                            pa.split('/')[-1], t[0], t[1], t[2], q.GetReal(), i[0], i[1], i[2]), flush=True)
                    else:
                        print('[PROBE]', pa, 'NOT FOUND', flush=True)
            except Exception as e:
                print('[PROBE] error:', e, flush=True)
        camera = self._env.scene["robot_pov_cam"]
        robot = self._env.scene["robot"]

        out = {}
        for name in ("demo_cam_front", "demo_cam_side"):
            if name in self._env.scene.keys():
                out[name] = self._env.scene[name].data.output["rgb"].clone()
        return "obs/robot_learning", {
            **out,
            "rgb": camera.data.output["rgb"].clone(),
            "joint_pos": robot.data.joint_pos.clone(),
            "joint_vel": robot.data.joint_vel.clone(),
            "root_pose": robot.data.root_link_pose_w.clone(),
            "raw_action": self._env.action_manager.action.clone(),
        }


@configclass
class RobotDatasetRecorderManagerCfg(ActionStateRecorderManagerCfg):
    record_robot_dataset = RecorderTermCfg(
        class_type=RobotDatasetRecorder,
    )


def install_robot_dataset_camera(env_cfg):
    """Attach an RGB camera to the physical G1 D435 link."""

    env_cfg.scene.robot_pov_cam = CameraCfg(
        prim_path=(
            "{ENV_REGEX_NS}/Robot/torso_link/d435_link/"
            "RobotLearningCamera"
        ),
        update_period=0.0,
        width=640,
        height=480,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=8.0,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            # D435 렌즈를 외장 앞으로 이동하고 G1 전방축(+X)에 정렬
            pos=(0.0, 0.0, 0.0),
            rot=(-0.4055798351764679, 0.40557971596717834, 0.5792279839515686, -0.5792279243469238),
            convention="opengl",
        ),
    )
