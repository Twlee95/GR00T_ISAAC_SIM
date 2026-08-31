#!/usr/bin/env bash
echo "=== 프로세스 ==="
pgrep -af \
"isaacteleop.cloudxr|sonic_xr_pipeline/sonic_xr_teleop.py" \
|| echo "실행 중인 프로세스 없음"

echo "=== 제어 구성/세션 ==="
grep -aE \
"upper_body_ik|lower_body_joint_pos|IsaacTeleop session started|XR_ERROR|Traceback" \
/tmp/isaaclab_teleop.log 2>/dev/null | tail -20

echo "=== 최신 CloudXR 상태 ==="
runtime=$(ls -t /root/.cloudxr/logs/cxr_server.*.log 2>/dev/null | head -1)
if [ -n "${runtime:-}" ]; then
    grep -iE \
    "Client connected|Input stream connected|Video stream connected|Input stream disconnected|Video stream disconnected" \
    "$runtime" | tail -12
fi
