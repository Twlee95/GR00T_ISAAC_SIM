#!/bin/bash
cd /workspace/wbc
pkill -9 -f run_sim_loop; pkill -9 -f deploy.sh; pkill -9 -f g1_deploy_onnx_ref; sleep 3
rm -f /workspace/wbc/sonic_sim_direct.mp4
cd /workspace/wbc/gear_sonic_deploy
export TensorRT_ROOT=/usr
printf "Y\n" | nohup bash deploy.sh sim > /tmp/deploy_log.txt 2>&1 &
sleep 45
cd /workspace/wbc
source .venv_sim/bin/activate
nohup python gear_sonic/scripts/run_sim_loop.py --enable-offscreen --no-enable-onscreen > /tmp/sim_log.txt 2>&1 &
echo "sim PID: $!"
sleep 5
python /workspace/wbc/play_dance.py > /tmp/send_log.txt 2>&1
echo "===== sim 생존 ====="; ps aux | grep run_sim_loop | grep -v grep || echo "sim 죽음"
echo "===== deploy 끝 8줄 ====="; tail -8 /tmp/deploy_log.txt
echo "===== send 끝 5줄 ====="; tail -5 /tmp/send_log.txt
echo "===== 영상 ====="; ls -la /workspace/wbc/sonic_sim_direct.mp4 2>/dev/null || echo "미저장"
