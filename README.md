# GR00T N1.7 + SONIC G1 Isaac Sim Setup

End-to-end pipeline for running GR00T N1.7 VLA with SONIC whole-body control on Unitree G1 in Isaac Sim.

## Requirements

- Ubuntu 24.04
- NVIDIA GPU (tested on RTX PRO 5000 Blackwell)
- NVIDIA Driver >= 535
- CUDA 12.x
- Docker 29.x
- NVIDIA Container Toolkit

## 1. Install Docker

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Install NVIDIA Container Toolkit

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
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

## 3. Clone Repository

```bash
git clone https://github.com/taewonlee95/Isaac-GR00T-N1.7.git
cd Isaac-GR00T-N1.7
```

Repository structure:
```
Isaac-GR00T-N1.7/
├── Isaac-GR00T/
│   ├── groot.g1.bridge/       ← Isaac Sim Extension (G1 bridge)
│   └── ...                    ← GR00T N1.7 code
└── GR00T-WholeBodyControl/
    ├── gear_sonic/            ← VLA inference code
    ├── gear_sonic_deploy/     ← SONIC C++ deploy code
    └── ...
```

## 4. Pull Docker Images

```bash
docker pull taewonlee95/sonic-build:g1-dds-working
docker pull taewonlee95/isaac-sim-n17:latest
docker pull taewonlee95/gr00t-n17:latest
```

Image roles:

| Image | Role |
|-------|------|
| `sonic-build:g1-dds-working` | SONIC C++ deploy + DDS bridge |
| `isaac-sim-n17:latest` | Isaac Sim 5.1.0 + G1 bridge extension |
| `gr00t-n17:latest` | GR00T N1.7 + VLA inference |

## 5. Communication Structure

```
[isaac-sim container]
  groot.g1.bridge Extension
    ZMQ 5559 PUB → robot state
    ZMQ 5558 SUB ← LowCmd
    ZMQ 5555 PUB → camera image

[sonic container]
  dds_bridge.py
    ZMQ 5559 SUB ← Isaac state
    DDS rt/lowstate PUB →
    DDS rt/lowcmd SUB ←
    ZMQ 5558 PUB → LowCmd

  g1_deploy_onnx_ref
    DDS rt/lowstate SUB ←
    ZMQ 5556 SUB ← motion_token (from VLA)
    DDS rt/lowcmd PUB →
    ZMQ 5567 PUB → g1_debug state

[gr00t container]
  run_vla_inference.py
    ZMQ 5555 SUB ← camera
    ZMQ 5567 SUB ← g1_debug
    ZMQ REQ 5550 ↔ GR00T PolicyServer
    ZMQ 5556 PUB → motion_token + hand joints
```

Overall flow:
```
Isaac Sim → state → dds_bridge → DDS → g1_deploy → LowCmd → dds_bridge → Isaac Sim
Isaac Sim → camera → VLA → motion_token → g1_deploy → LowCmd
```

## 6. Run

### Step 1: Start Containers

```bash
# SONIC container
docker run -dit \
  --name sonic \
  --network host \
  --gpus all \
  taewonlee95/sonic-build:g1-dds-working \
  tail -f /dev/null

# GR00T container
docker run -dit \
  --name gr00t \
  --network host \
  --gpus all \
  taewonlee95/gr00t-n17:latest \
  tail -f /dev/null

# Isaac Sim container
# Adjust workspace path to match your environment
WORKSPACE=/home/$USER/workspace/Isaac-GR00T-N1.7

docker run -dit \
  --name isaac-sim \
  --network host \
  --gpus all \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -v $WORKSPACE/Isaac-GR00T/groot.g1.bridge:/isaac-sim/extsUser/groot.g1.bridge \
  -v $WORKSPACE/GR00T-WholeBodyControl:/workspace/gr00t/GR00T-WholeBodyControl \
  taewonlee95/isaac-sim-n17:latest \
  bash -c "cd /isaac-sim && ./isaac-sim.streaming.sh"
```

Verify:
```bash
docker ps
```

Expected:
```
sonic       Up
gr00t       Up
isaac-sim   Up
```

### Step 2: Start DDS Bridge

```bash
docker exec -d sonic bash -c '
LD_LIBRARY_PATH=/workspace/thirdparty/unitree_sdk2/thirdparty/lib:$LD_LIBRARY_PATH \
CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"false\"/></Interfaces></General></Domain></CycloneDDS>" \
PYTHONUNBUFFERED=1 \
python3 /bridge/dds_bridge.py > /tmp/dds_bridge.log 2>&1
'
```

Verify:
```bash
sleep 3 && docker exec sonic tail -20 /tmp/dds_bridge.log
```

Expected:
```
[DDS Bridge v3] 시작!
[DDS Bridge v3] Isaac state SUB: port 5559
[DDS Bridge v3] LowCmd ZMQ PUB: port 5558
[DDS Bridge v3] DDS 초기화 완료
[DDS Bridge v3] DDS LowState PUB: rt/lowstate
[DDS Bridge v3] DDS LowCmd SUB: rt/lowcmd
[DDS Bridge v3] 메인 루프 시작!
```

### Step 3: Start GR00T PolicyServer

```bash
docker exec -d gr00t bash -c '
cd /workspace/gr00t && \
uv run python gr00t/eval/run_gr00t_server.py \
  --model-path nvidia/GR00T-N1.7-3B \
  --embodiment-tag UNITREE_G1_SONIC \
  --device cuda:0 \
  --port 5550 > /tmp/gr00t_server.log 2>&1
'
```

Wait ~60 seconds for model to load, then verify:
```bash
docker exec gr00t tail -20 /tmp/gr00t_server.log
```

### Step 4: Start VLA Inference

```bash
docker exec -d gr00t bash -c '
cd /workspace/GR00T-WholeBodyControl && \
python gear_sonic/scripts/run_vla_inference.py \
  --host localhost \
  --port 5550 > /tmp/vla_inference.log 2>&1
'
```

Verify:
```bash
docker exec gr00t tail -20 /tmp/vla_inference.log
```

### Step 5: Start G1 Deploy

```bash
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

Expected:
```
[DEBUG] G1Deploy object created successfully!
Init Done
[DEBUG] LowStateHandler called: 100
```

## 7. Verify Full Pipeline

```bash
# Isaac Sim bridge status
docker logs isaac-sim 2>&1 | grep -E "G1 Bridge|URDF|초기화|LowCmd" | tail -20

# DDS bridge status
docker exec sonic tail -10 /tmp/dds_bridge.log

# SONIC processes
docker exec sonic pgrep -af 'dds_bridge|g1_deploy'

# GR00T server
docker exec gr00t tail -10 /tmp/gr00t_server.log

# VLA inference
docker exec gr00t tail -10 /tmp/vla_inference.log
```

## 8. Stop

```bash
docker exec sonic bash -c "pkill -f '[d]ds_bridge.py' || true"
docker exec sonic pkill -f g1_deploy_onnx_ref || true
docker stop sonic gr00t isaac-sim
```

## 9. Restart from Clean State

```bash
docker rm -f sonic gr00t isaac-sim
# Then repeat from Step 6
```

## Port Reference

| Port | Direction | Content |
|------|-----------|---------|
| 5550 | REQ/REP | GR00T PolicyServer |
| 5555 | PUB | Isaac Sim camera → VLA |
| 5556 | PUB | VLA motion_token → SONIC |
| 5558 | PUB | dds_bridge LowCmd → Isaac Sim |
| 5559 | PUB | Isaac Sim state → dds_bridge |
| 5567 | PUB | SONIC g1_debug state |

## Key Modifications

### 1. CycloneDDS loopback configuration
Both `dds_bridge.py` and `g1_deploy_onnx_ref` use identical DDS config:
```
CYCLONEDDS_URI="<CycloneDDS><Domain Id="any"><General><Interfaces>
  <NetworkInterface name="lo" multicast="false"/>
</Interfaces></General></Domain></CycloneDDS>"
```
Required for loopback DDS communication between processes in the same container.

### 2. Python cyclonedds built against SONIC libddsc.so
`dds_bridge.py` uses cyclonedds Python bindings compiled against:
```
/workspace/thirdparty/unitree_sdk2/thirdparty/lib/x86_64/libddsc.so.0
```
This ensures Python and C++ share the same DDS library instance.

### 3. dds_bridge.py
Translates between Isaac Sim ZMQ and Unitree DDS:
- Isaac Sim ZMQ 5559 → DDS `rt/lowstate`
- DDS `rt/lowcmd` → ZMQ 5558 → Isaac Sim

### 4. G1 bridge Extension (groot.g1.bridge)
Isaac Sim Extension that:
- Loads G1 URDF (29DOF body + 14DOF hands)
- Publishes robot state via ZMQ 5559
- Publishes camera via ZMQ 5555
- Receives LowCmd via ZMQ 5558 and applies to G1 joints

### 5. ZMQ output port conflict fix
`g1_deploy_onnx_ref` uses `--zmq-out-port 5567` instead of default 5557 to avoid conflict with other processes.