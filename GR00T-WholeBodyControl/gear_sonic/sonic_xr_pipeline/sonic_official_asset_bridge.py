"""Official G1 visuals with SONIC body control."""

from isaaclab_assets.robots.unitree import G1_29DOF_CFG
from gear_sonic.envs.manager_env.robots import g1


SONIC_BODY_JOINTS = [
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


def install(config):
    """Use official G1 USD while preserving SONIC dynamics and actuators."""

    sonic_cfg = g1.G1_CYLINDER_MODEL_12_DEX_CFG

    # Official mesh/USD + SONIC initial state, gains and actuator definitions.
    g1.G1_CYLINDER_MODEL_12_DEX_CFG = sonic_cfg.replace(
        spawn=G1_29DOF_CFG.spawn,
    )

    # Control only the 29 joints used by the SONIC checkpoint.
    config.manager_env.actions.joint_pos.joint_names = SONIC_BODY_JOINTS

    print(
        "[SONIC-OFFICIAL] official G1 USD installed; "
        "29 SONIC body joints enabled",
        flush=True,
    )
