import os
os.environ["MUJOCO_GL"] = "egl"
import sys, types
fake_pynput = types.ModuleType("pynput")
fake_kb = types.ModuleType("pynput.keyboard")
class _DL:
    def __init__(self,*a,**k): pass
    def start(self): pass
    daemon=True
fake_kb.Listener=_DL; fake_pynput.keyboard=fake_kb
sys.modules["pynput"]=fake_pynput; sys.modules["pynput.keyboard"]=fake_kb

sys.path.insert(0, "/workspace/wbc/decoupled_wbc/sim2mujoco/scripts")
import numpy as np, torch, mujoco, imageio
import run_mujoco_gear_wbc as G
CFG = "/workspace/wbc/decoupled_wbc/sim2mujoco/resources/robots/g1"
G.CONFIG_PATH = CFG
G.GearWbcController.keyboard_listener = lambda self,c,cfg: None
ctrl = G.GearWbcController(CFG)
ctrl.control_dict["loco_cmd"] = np.array([0.5, 0.0, 0.0], dtype=np.float32)

renderer = mujoco.Renderer(ctrl.model, height=480, width=640)
frames=[]; na=ctrl.config["num_actions"]; nj=ctrl.n_joints
for step in range(3000):
    leg_tau = ctrl.pd_control(ctrl.target_dof_pos, ctrl.data.qpos[7:7+na],
        ctrl.config["kps"], np.zeros_like(ctrl.config["kps"]),
        ctrl.data.qvel[6:6+na], ctrl.config["kds"])
    ctrl.data.ctrl[:na]=leg_tau
    if nj>na:
        arm_tau = ctrl.pd_control(np.zeros(nj-na,dtype=np.float32),
            ctrl.data.qpos[7+na:7+nj], np.full(nj-na,100.0),
            np.zeros(nj-na), ctrl.data.qvel[6+na:6+nj], np.full(nj-na,0.5))
        ctrl.data.ctrl[na:]=arm_tau
    mujoco.mj_step(ctrl.model, ctrl.data)
    ctrl.counter+=1
    if ctrl.counter % ctrl.config["control_decimation"]==0:
        so,_ = ctrl.compute_observation(ctrl.data, ctrl.config, ctrl.action, ctrl.control_dict, nj)
        ctrl.obs_history.append(so)
        for i,h in enumerate(ctrl.obs_history):
            ctrl.obs[i*ctrl.single_obs_dim:(i+1)*ctrl.single_obs_dim]=h
        ot = torch.from_numpy(ctrl.obs).unsqueeze(0)
        if np.linalg.norm(np.array(ctrl.control_dict["loco_cmd"]))<=0.05:
            ctrl.action = ctrl.policy(ot).cpu().detach().numpy().squeeze()
        else:
            ctrl.action = ctrl.walk_policy(ot).cpu().detach().numpy().squeeze()
        ctrl.target_dof_pos = ctrl.action*ctrl.config["action_scale"]+ctrl.config["default_angles"]
    if step%10==0:
        renderer.update_scene(ctrl.data)
        frames.append(renderer.render())
imageio.mimsave("/workspace/wbc/walk_test.mp4", frames, fps=20)
print(f"SAVED frames={len(frames)} final_height={ctrl.data.qpos[2]:.3f}")
