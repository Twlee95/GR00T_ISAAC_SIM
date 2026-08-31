"""Demo config: no auto-termination, rock + Part3, manual reset."""

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils.configclass import configclass

from . import mdp as locomanip_mdp
from .locomanipulation_g1_env_cfg import LocomanipulationG1EnvCfg


@configclass
class DemoTerminationsCfg:
    time_out = DoneTerm(func=locomanip_mdp.time_out, time_out=True)


@configclass
class DemoG1EnvCfg(LocomanipulationG1EnvCfg):
    terminations: DemoTerminationsCfg = DemoTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 3600.0
        self.sim.device = 'cuda:0'
        self.scene.robot.actuators['arms'].effort_limit = {
            '.*_shoulder_pitch_joint': 25.0,
            '.*_shoulder_roll_joint': 25.0,
            '.*_shoulder_yaw_joint': 25.0,
            '.*_elbow_joint': 25.0,
            '.*_wrist_roll_joint': 25.0,
            '.*_wrist_pitch_joint': 5.0,
            '.*_wrist_yaw_joint': 5.0,
        }
        self.scene.robot.actuators['hands'].effort_limit = {
            '.*_thumb_0_joint': 2.45,
            '.*_thumb_1_joint': 1.4,
            '.*_thumb_2_joint': 1.4,
            '.*_index_.*': 1.4,
            '.*_middle_.*': 1.4,
        }

        self.scene.packing_table = None

        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.7, 1.76, 0.85], rot=[0, 0, 0, 1]),
            spawn=UsdFileCfg(
                usd_path="/home/taewon/workspace/saved_usd/rock/namaqualand_boulder_05_4k.usdc",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            ),
        )

        self.scene.object2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object2",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.7, 1.76, 0.85], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path="/home/taewon/workspace/saved_usd/chemlab/Part3/Part3_clean.usd",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            ),
        )

        self.scene.object3 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object3",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.7, 1.76, 0.92], rot=[0, 0, 0, 1]),
            spawn=UsdFileCfg(
                usd_path="/home/taewon/workspace/saved_usd/chemlab/Part3/Part3_lid.usd",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            ),
        )
