"""Apply copied official Teleoperation visuals to SONIC."""

from copy import deepcopy

import isaaclab.sim as sim_utils
from gear_sonic.envs.manager_env import modular_tracking_env_cfg as sonic_env
from gear_sonic.sonic_xr_pipeline.official_task.locomanipulation_g1_env_cfg import (
    LocomanipulationG1SceneCfg,
)


def install():
    if getattr(sonic_env, "_SONIC_OFFICIAL_INSTALLED", False):
        return

    original_init = sonic_env.MySceneCfg.__init__

    def official_scene_init(self, config):
        original_init(self, config)

        official = LocomanipulationG1SceneCfg()
        self.packing_table = deepcopy(official.packing_table)
        self.object = deepcopy(official.object)
        self.light = deepcopy(official.light)

        if hasattr(self, "sky_light"):
            self.sky_light = None

        if hasattr(self, "terrain") and hasattr(self.terrain, "visual_material"):
            self.terrain.visual_material = sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.35, 0.35, 0.35),
                roughness=0.8,
            )

        print(
            "[SONIC-OFFICIAL] official scene visuals installed",
            flush=True,
        )

    sonic_env.MySceneCfg.__init__ = official_scene_init
    sonic_env._SONIC_OFFICIAL_INSTALLED = True

    original_override = sonic_env.ModularTrackingEnvCfg.override_settings

    def official_render_settings(self):
        original_override(self)
        self.sim.render_interval = 2
        self.sim.render.antialiasing_mode = "DLSS"

    sonic_env.ModularTrackingEnvCfg.override_settings = official_render_settings
