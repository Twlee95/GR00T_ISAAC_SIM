"""Third-person cameras for demo recording."""
import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg


def install_demo_cameras(env_cfg):
    env_cfg.scene.demo_cam_front = CameraCfg(
        prim_path="{ENV_REGEX_NS}/DemoCamFront",
        update_period=0.0,
        width=1280,
        height=720,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 50.0)),
        offset=CameraCfg.OffsetCfg(
            pos=(-2.3099, -1.8005, 2.7169),
            rot=(-0.1300, 0.1978, 0.5446, 0.8046),
            convention="world",
        ),
    )

    env_cfg.scene.demo_cam_side = CameraCfg(
        prim_path="{ENV_REGEX_NS}/DemoCamSide",
        update_period=0.0,
        width=1280,
        height=720,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 50.0)),
        offset=CameraCfg.OffsetCfg(
            pos=(2.3251, -1.8839, 2.7682),
            rot=(-0.2172, 0.0940, 0.8969, 0.3735),
            convention="world",
        ),
    )

def probe_scene():
    import omni.usd
    from pxr import UsdGeom, Usd
    st = omni.usd.get_context().get_stage()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    for path in ['/World/envs/env_0/Kitchen/model_table_03',
                 '/World/envs/env_0/Kitchen/model_table_04',
                 '/World/envs/env_0/Kitchen/Camera_01',
                 '/World/envs/env_0/Kitchen/Camera_02',
                 '/World/envs/env_0/Object',
                 '/World/envs/env_0/Robot']:
        pr = st.GetPrimAtPath(path)
        if pr:
            m = UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            print('[PROBE] %-50s (%.2f, %.2f, %.2f)' % (path.split('/')[-1], t[0], t[1], t[2]), flush=True)
        else:
            print('[PROBE] %s NOT FOUND' % path, flush=True)
