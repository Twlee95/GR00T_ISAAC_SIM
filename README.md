# GR00T N1.7 + SONIC G1 Isaac Sim Setup

End-to-end setup for running Unitree G1 in Isaac Sim with SONIC whole-body control.

Current verified status:

```text
Verified:
Isaac Sim ↔ DDS Bridge ↔ SONIC g1_deploy ↔ LowCmd ↔ Isaac Sim

Not yet verified:
Full GR00T VLA → SONIC motion_token pipeline
```

---

## Requirements

- Ubuntu 24.04
- NVIDIA GPU
- NVIDIA Driver >= 535
- CUDA 12.x
- Docker
- NVIDIA Container Toolkit
- Docker images:
  - `taewonlee95/isaac-sim-n17:latest`
  - `taewonlee95/sonic-build:g1-dds-working`
  - `taewonlee95/gr00t-n17:latest`

---

## 1. Install Docker

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker
```

---

## 2. Install NVIDIA Container Toolkit

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

---

## 3. Clone Repository

```bash
mkdir -p /home/$USER/workspace
cd /home/$USER/workspace

git clone https://github.com/Twlee95/GR00T_ISAAC_SIM.git Isaac-GR00T-N1.7
cd Isaac-GR00T-N1.7
```

Expected structure:

```text
Isaac-GR00T-N1.7/
├── Isaac-GR00T/
│   ├── groot.g1.bridge/
│   └── ...
└── GR00T-WholeBodyControl/
    ├── gear_sonic/
    ├── gear_sonic_deploy/
    └── ...
```

Set workspace path:

```bash
export WORKSPACE=/home/$USER/workspace/Isaac-GR00T-N1.7
```

---

## 4. Pull Docker Images

```bash
docker pull taewonlee95/isaac-sim-n17:latest
docker pull taewonlee95/sonic-build:g1-dds-working
docker pull taewonlee95/gr00t-n17:latest
```

Image roles:

| Image | Role |
|---|---|
| `taewonlee95/isaac-sim-n17:latest` | Isaac Sim 5.1.0 + G1 bridge extension |
| `taewonlee95/sonic-build:g1-dds-working` | SONIC deploy runtime + DDS bridge runtime |
| `taewonlee95/gr00t-n17:latest` | GR00T / VLA runtime environment |

---

## 5. Communication Structure

```text
[isaac-sim container]
  G1 Bridge Extension
    ZMQ 5559 PUB → Isaac robot state
    ZMQ 5558 SUB ← LowCmd from sonic
    ZMQ 5555 PUB → camera image

[sonic container]
  dds_bridge.py
    ZMQ 5559 SUB ← Isaac robot state
    DDS rt/lowstate PUB →
    DDS rt/lowcmd SUB ←
    ZMQ 5558 PUB → LowCmd to Isaac Sim

  g1_deploy_onnx_ref
    DDS rt/lowstate SUB ←
    SONIC policy / encoder / planner
    DDS rt/lowcmd PUB →
    ZMQ 5567 PUB → g1_debug/state

[gr00t container]
  Reserved for GR00T PolicyServer / VLA inference.
  The container starts successfully, but full VLA is not verified yet.
```

Verified control loop:

```text
Isaac Sim
→ ZMQ 5559 robot state
→ dds_bridge.py
→ DDS rt/lowstate
→ g1_deploy_onnx_ref
→ DDS rt/lowcmd
→ dds_bridge.py
→ ZMQ 5558 LowCmd
→ Isaac Sim joint application
```

---

## 6. Start Three Containers

Remove old containers first:

```bash
docker rm -f isaac-sim sonic gr00t 2>/dev/null || true
```

---

### 6.1 Start Isaac Sim

```bash
docker run -d \
  --name isaac-sim \
  --gpus all \
  --network host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -v $WORKSPACE/Isaac-GR00T/groot.g1.bridge:/isaac-sim/extsUser/groot.g1.bridge \
  -v $WORKSPACE/GR00T-WholeBodyControl:/workspace/gr00t/GR00T-WholeBodyControl \
  --entrypoint bash \
  taewonlee95/isaac-sim-n17:latest \
  -c "cd /isaac-sim && ./isaac-sim.streaming.sh"
```

Check Isaac Sim:

```bash
docker logs isaac-sim 2>&1 | grep -E "G1 Bridge|Timeline|URDF|초기화|ego_view|step" | tail -80
```

Expected:

```text
[G1 Bridge] v5 시작!
[G1 Bridge] Timeline play!
[G1 Bridge] URDF 로드: /World/g1_29dof_with_hand
[G1 Bridge] 바닥 추가!
[G1 Bridge] 초기화 완료! DOF=43
[G1 Bridge] G1 root pose / home joint 설정 완료!
[G1 Bridge] ego_view 카메라 초기화 완료!
```

---

### 6.2 Start SONIC

Important: the SONIC container must mount both `gear_sonic_deploy` and `groot.g1.bridge`.

```bash
docker run -dit \
  --name sonic \
  --gpus all \
  --network host \
  -v $WORKSPACE/GR00T-WholeBodyControl/gear_sonic_deploy:/workspace \
  -v $WORKSPACE/Isaac-GR00T/groot.g1.bridge:/bridge \
  -e TensorRT_ROOT=/usr \
  taewonlee95/sonic-build:g1-dds-working \
  tail -f /dev/null
```

Check required files:

```bash
docker exec sonic ls /bridge/dds_bridge.py
docker exec sonic ls /workspace/target/release/g1_deploy_onnx_ref
```

Expected:

```text
/bridge/dds_bridge.py
/workspace/target/release/g1_deploy_onnx_ref
```

---

### 6.3 Start GR00T container

This starts the GR00T/VLA environment container.  
The full VLA runtime is not verified yet because the required `UNITREE_G1_SONIC` finetuned checkpoint is missing.

```bash
docker run -dit \
  --name gr00t \
  --network host \
  --gpus all \
  -v $WORKSPACE/Isaac-GR00T:/workspace/gr00t \
  -v $WORKSPACE/GR00T-WholeBodyControl:/workspace/GR00T-WholeBodyControl \
  -e HF_HOME=/workspace/gr00t/checkpoints \
  taewonlee95/gr00t-n17:latest \
  tail -f /dev/null
```

Check containers:

```bash
docker ps
```

Expected:

```text
isaac-sim
sonic
gr00t
```

---

## 7. Start Verified SONIC Runtime

### 7.1 Start DDS Bridge

```bash
docker exec sonic bash -c "pkill -f '[d]ds_bridge.py' || true"

docker exec -d sonic bash -c '
LD_LIBRARY_PATH=/workspace/thirdparty/unitree_sdk2/thirdparty/lib:$LD_LIBRARY_PATH \
CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces></General></Domain></CycloneDDS>" \
PYTHONUNBUFFERED=1 \
python3 /bridge/dds_bridge.py > /tmp/dds_bridge.log 2>&1
'
```

Check:

```bash
docker exec sonic tail -40 /tmp/dds_bridge.log
```

Expected:

```text
[DDS Bridge v3] 시작!
[DDS Bridge v3] Isaac state SUB: port 5559
[DDS Bridge v3] LowCmd ZMQ PUB: port 5558
[DDS Bridge v3] DDS 초기화 완료
[DDS Bridge v3] DDS LowState PUB: rt/lowstate
[DDS Bridge v3] DDS LowCmd SUB: rt/lowcmd
[DDS Bridge v3] 메인 루프 시작!
```

---

### 7.2 Start SONIC G1 Deploy

```bash
docker exec sonic pkill -f g1_deploy_onnx_ref || true

docker exec -it sonic bash -c '
cd /workspace
CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces></General></Domain></CycloneDDS>" \
./target/release/g1_deploy_onnx_ref lo \
policy/release/model_decoder.onnx \
reference/example \
--planner-file planner/target_vel/V2/planner_sonic.onnx \
--obs-config policy/release/observation_config.yaml \
--encoder-file policy/release/model_encoder.onnx \
--disable-crc-check \
--zmq-out-port 5567
'
```

Expected logs:

```text
[DEBUG] Program starting...
[DEBUG] Arguments validated...
[DEBUG] Creating G1Deploy object...
[DEBUG] Before MotionSwitcherClient
[DEBUG] MotionSwitcherClient created
[DEBUG] MotionSwitcherClient timeout set
[DEBUG] MotionSwitcherClient init done
[DEBUG] Before CheckMode
[DEBUG] After MotionSwitcherClient block
✓ Motion data loaded successfully!
✓ Policy model loaded successfully!
✓ Encoder model loaded successfully!
✓ TensorRT planner model loaded successfully!
[DEBUG] G1Deploy object created successfully!
Init Done
[DEBUG] LowStateHandler called: 100
```

Keep this terminal open.

---

## 8. Verify LowCmd Loop

In another terminal:

```bash
docker logs isaac-sim 2>&1 | grep "LowCmd 적용" | tail -10
```

Expected:

```text
[G1 Bridge] LowCmd 적용: len(motor_cmds)=29, target[0:10]=[-0.312  0.     0.     0.669 -0.363  0.    -0.312  0.     0.     0.669]
```

This confirms:

```text
Isaac Sim state reaches SONIC.
SONIC g1_deploy receives LowState.
SONIC generates 29-DOF LowCmd.
LowCmd returns to Isaac Sim.
Isaac Sim applies the joint command.
```

Process check:

```bash
docker exec sonic pgrep -af 'dds_bridge|g1_deploy'
```

Expected:

```text
python3 /bridge/dds_bridge.py
./target/release/g1_deploy_onnx_ref ...
```

---

## 9. Current Behavior

The current SONIC output can remain nearly constant:

```text
target[0:10]=[-0.312  0.     0.     0.669 -0.363  0.    -0.312  0.     0.     0.669]
```

This is expected in the current verified setup.

Reason:

```text
g1_deploy starts with a reference motion loaded but paused at frame 0.
The system verifies state → LowCmd connectivity, not dynamic motion playback.
```

Observed log:

```text
Started with motion: dance_in_da_party_001__A464 (paused at frame 0)
```

So:

```text
Verified:
LowState input and LowCmd output loop.

Not verified:
Dynamic motion progression or VLA-driven motion_token control.
```

---

## 10. VLA / GR00T Status

The GR00T container starts, but full VLA is not yet verified.

Attempted PolicyServer command:

```bash
uv run python gr00t/eval/run_gr00t_server.py \
  --model-path nvidia/GR00T-N1.7-3B \
  --embodiment-tag UNITREE_G1_SONIC
```

This fails because:

```text
nvidia/GR00T-N1.7-3B is the base model.
UNITREE_G1_SONIC is a posttrain tag and requires a finetuned checkpoint.
```

Observed error:

```text
Hint: 'UNITREE_G1_SONIC' is a posttrain tag that requires a finetuned checkpoint, not the base model.
```

Current checkpoint search showed only:

```text
nvidia/GR00T-N1.7-3B base model
nvidia/Cosmos-Reason2-2B
motionbricks checkpoints
SMPL-related pt/pth files
```

Missing:

```text
UNITREE_G1_SONIC finetuned GR00T checkpoint
```

To run the full VLA pipeline later, use:

```bash
uv run python gr00t/eval/run_gr00t_server.py \
  --model-path /path/to/unitree_g1_sonic_finetuned_checkpoint \
  --embodiment-tag UNITREE_G1_SONIC \
  --device cuda:0 \
  --port 5550
```

---

## 11. Optional: VLA Inference Runner Environment

The VLA inference runner is not the PolicyServer.  
It is a client that reads camera/state and sends requests to the PolicyServer.

Previous working environment pattern:

```bash
docker exec -it gr00t bash
cd /workspace/GR00T-WholeBodyControl
bash install_scripts/install_inference.sh
source .venv_inference/bin/activate
PYTHONPATH=/workspace/gr00t:$PYTHONPATH \
python gear_sonic/scripts/run_vla_inference.py \
  --host localhost \
  --port 5550 \
  --state-zmq-port 5567 \
  --action-zmq-port 5556 \
  --camera-port 5555
```

This requires the GR00T PolicyServer to be running on port `5550`.

---

## 12. Port Reference

| Port | Direction | Content |
|---|---|---|
| 5550 | GR00T PolicyServer | VLA inference ↔ PolicyServer |
| 5555 | Isaac Sim → VLA | Camera image |
| 5556 | VLA → SONIC | motion_token + hand joints |
| 5558 | SONIC → Isaac Sim | LowCmd |
| 5559 | Isaac Sim → SONIC | Robot state |
| 5567 | SONIC → VLA | g1_debug/state |

---

## 13. Key Modifications

### 13.1 Isaac Sim G1 bridge

The Isaac Sim extension:

```text
Isaac-GR00T/groot.g1.bridge
```

does the following:

```text
Loads G1 URDF.
Publishes robot state through ZMQ 5559.
Publishes camera through ZMQ 5555.
Receives LowCmd through ZMQ 5558.
Applies LowCmd to Isaac Sim G1 joints.
```

---

### 13.2 G1 root pose initialization

The G1 bridge explicitly initializes the robot root pose:

```python
self._robot.set_world_pose(
    position=np.array([0.0, 0.0, 0.80], dtype=np.float32),
    orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
)
```

Reason:

```text
The G1 robot must spawn near the world origin and above the ground.
```

---

### 13.3 Home joint initialization

The bridge initializes the first body joints:

```python
home_joints = np.zeros(self._robot.num_dof, dtype=np.float32)

if self._robot.num_dof >= 29:
    home_joints[0] = -0.312
    home_joints[1] = 0.0
    home_joints[2] = 0.0

self._robot.set_joint_positions(home_joints)
```

---

### 13.4 LowCmd debug log

The bridge prints LowCmd application:

```python
print(
    f"[G1 Bridge] LowCmd 적용: "
    f"len(motor_cmds)={len(motor_cmds)}, "
    f"target[0:10]={np.round(target[:10], 3)}"
)
```

This confirms whether all 29 body commands are received.

---

### 13.5 DDS loopback configuration

Both `dds_bridge.py` and `g1_deploy_onnx_ref` use:

```bash
CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces></General></Domain></CycloneDDS>"
```

Reason:

```text
The simulated DDS participants communicate through loopback.
Both Python DDS bridge and C++ SONIC deploy must use the same DDS interface.
```

---

### 13.6 SONIC container volume mounts

The SONIC container must mount:

```text
$WORKSPACE/GR00T-WholeBodyControl/gear_sonic_deploy → /workspace
$WORKSPACE/Isaac-GR00T/groot.g1.bridge → /bridge
```

Reason:

```text
/bridge/dds_bridge.py is required for ZMQ ↔ DDS translation.
/workspace/target/release/g1_deploy_onnx_ref is required for SONIC control.
```

---

## 14. Stop

Stop runtime processes:

```bash
docker exec sonic bash -c "pkill -f '[d]ds_bridge.py' || true"
docker exec sonic pkill -f g1_deploy_onnx_ref || true
```

Stop containers:

```bash
docker stop isaac-sim sonic gr00t
```

Remove containers:

```bash
docker rm -f isaac-sim sonic gr00t
```

---

## 15. Large File Policy

Large model and engine files should not be committed to git:

```text
*.ckpt
*.pt
*.pth
```