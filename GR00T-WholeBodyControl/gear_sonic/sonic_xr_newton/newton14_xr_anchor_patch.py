"""Synchronize Newton transforms immediately before XR anchor reads."""

def install():
    from isaaclab.sim.utils.stage import get_current_stage, get_current_stage_id
    from isaaclab_newton.assets.articulation.articulation_data import (
        SimulationManager,
    )
    from isaaclab_newton.physics.newton_manager import NewtonManager
    from isaaclab_teleop.xr_anchor_utils import XrAnchorSynchronizer

    if getattr(XrAnchorSynchronizer, "_newton14_xr_installed", False):
        return

    original = XrAnchorSynchronizer.sync_headset_to_anchor

    def synchronized(self):
        # XR이 pelvis를 읽기 직전에 Newton 상태를 Fabric에 반영한다.
        NewtonManager.sync_transforms_to_usd()
        original(self)

        count = getattr(self, "_newton14_xr_count", 0) + 1
        self._newton14_xr_count = count
        if count % 120:
            return

        try:
            import usdrt
            from usdrt import Rt

            path = self._xr_cfg.anchor_prim_path
            stage = usdrt.Usd.Stage.Attach(get_current_stage_id())
            prim = stage.GetPrimAtPath(path)
            matrix = (
                Rt.Xformable(prim)
                .GetFabricHierarchyWorldMatrixAttr()
                .Get()
            )
            fabric_pos = tuple(float(v) for v in matrix.ExtractTranslation())

            anchor_prim = stage.GetPrimAtPath(
                self._xr_anchor_headset_path
            )
            anchor_matrix = (
                Rt.Xformable(anchor_prim)
                .GetFabricHierarchyWorldMatrixAttr()
                .Get()
            )
            applied_anchor_pos = tuple(
                float(v)
                for v in anchor_matrix.ExtractTranslation()
            )

            model = SimulationManager.get_model()
            state = SimulationManager.get_state_0()
            labels = list(
                getattr(model, "body_label", None)
                or getattr(model, "body_key", None)
                or []
            )

            body_pos = None
            if path in labels:
                index = labels.index(path)
                body_pos = tuple(
                    float(v) for v in state.body_q.numpy()[index][:3]
                )

            anchor = self.get_world_transform()
            anchor_pos = (
                tuple(float(v) for v in anchor[0])
                if anchor is not None else None
            )

            from pxr import UsdGeom
            import carb

            meters_per_unit = float(
                UsdGeom.GetStageMetersPerUnit(get_current_stage())
            )

            settings = carb.settings.get_settings()
            xr_scales = {
                key: settings.get(key)
                for key in (
                    "/persistent/xr/worldScale",
                    "/xrstage/worldScale",
                    "/persistent/xr/stageUnitsPerMeter",
                    "/xrstage/stageUnitsPerMeter",
                )
            }

            print(
                "[NEWTON14-XR] "
                f"meters_per_unit={meters_per_unit} "
                f"xr_scales={xr_scales} "
                f"body_pelvis={body_pos} "
                f"fabric_pelvis={fabric_pos} "
                f"target_anchor={anchor_pos} "
                f"applied_anchor={applied_anchor_pos} "
                f"anchor_path={self._xr_anchor_headset_path}",
                flush=True,
            )
        except Exception as exc:
            print(f"[NEWTON14-XR] diagnostic failed: {exc!r}", flush=True)

    XrAnchorSynchronizer.sync_headset_to_anchor = synchronized
    XrAnchorSynchronizer._newton14_xr_installed = True
    print("[NEWTON14-XR] pre-anchor synchronization installed", flush=True)
