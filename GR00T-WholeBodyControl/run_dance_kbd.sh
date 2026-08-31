#!/bin/bash
cd /workspace/wbc
pkill -9 -f run_sim_loop; pkill -9 -f deploy.sh; pkill -9 -f g1_deploy_onnx_ref; sleep 3
rm -f /workspace/wbc/sonic_sim_direct.mp4
source .venv_sim/bin/activate
nohup python gear_sonic/scripts/run_sim_loop.py --enable-offscreen --no-enable-onscreen > /tmp/sim_log.txt 2>&1 &
echo "sim PID: $! (발행 대기)"
sleep 10
ps -p $! > /dev/null && echo "sim 생존" || { echo "sim 죽음"; tail -5 /tmp/sim_log.txt; }
cd /workspace/wbc/gear_sonic_deploy
export TensorRT_ROOT=/usr
bash deploy.sh --input-type keyboard sim
