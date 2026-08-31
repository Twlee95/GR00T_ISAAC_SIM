#!/usr/bin/env bash
set -euo pipefail

cd /workspace/wbc

pkill -TERM -f "[s]onic_xr_v2/sonic_xr_teleop.py" 2>/dev/null || true
pkill -TERM -f "[i]saacteleop.cloudxr" 2>/dev/null || true

# 기존 프로세스 종료를 최대 10초 확인
for _ in $(seq 1 40); do
    if ! pgrep -f "[i]saacteleop.cloudxr" >/dev/null; then
        break
    fi
    sleep 0.25
done

pkill -KILL -f "[i]saacteleop.cloudxr" 2>/dev/null || true

rm -f /root/.cloudxr/run/ipc_cloudxr
rm -f /root/.cloudxr/run/cloudxr.env
: > /tmp/cloudxr.log
: > /tmp/v2_fullbody.log

python3 -u -m isaacteleop.cloudxr \
    --accept-eula \
    --cloudxr-env-config /share/pico.env \
    > /tmp/cloudxr.log 2>&1 &

cloudxr_pid=$!
echo "$cloudxr_pid" > /tmp/pico_cloudxr.pid
echo "[1/2] CloudXR 시작 중..."

ready=0
for _ in $(seq 1 720); do
    if [[ -s /root/.cloudxr/run/cloudxr.env ]]; then
        ready=1
        break
    fi

    if ! kill -0 "$cloudxr_pid" 2>/dev/null; then
        echo "CloudXR 시작 실패"
        tail -80 /tmp/cloudxr.log
        exit 1
    fi

    sleep 0.25
done

if [[ "$ready" != "1" ]]; then
    echo "CloudXR 준비 시간 초과"
    tail -80 /tmp/cloudxr.log
    kill "$cloudxr_pid" 2>/dev/null || true
    exit 1
fi

source /root/.cloudxr/run/cloudxr.env
export XDG_RUNTIME_DIR=/root/.cloudxr/run
export ISAACLAB_CXR_SKIP_AUTOLAUNCH=1
export PYTHONPATH=/workspace/wbc

echo "[2/2] Isaac Teleop 시작"

# Python 출력은 파일에 직접 기록한다. 화면 연결이 끊겨도 Python은 유지된다.
python3 -u gear_sonic/sonic_xr_v2/sonic_xr_teleop.py \
    --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
    --xr \
    --enable_cameras \
    --headless \
    --no-auto_launch_cloudxr \
    --num_demos 0 \
    --dataset_file ./datasets/teleop_v2_fullbody.hdf5 \
    >> /tmp/v2_fullbody.log 2>&1 &

teleop_pid=$!
echo "$teleop_pid" > /tmp/pico_v2.pid
echo "Isaac Teleop PID=$teleop_pid"

# 화면 출력용 tail은 Python과 연결하지 않는다.
tail -n 0 -F /tmp/v2_fullbody.log &
tail_pid=$!

set +e
wait "$teleop_pid"
status=$?
set -e

kill "$tail_pid" 2>/dev/null || true
wait "$tail_pid" 2>/dev/null || true

echo "$status" > /tmp/pico_v2.exit
echo "[EXIT] Isaac Teleop exit status=$status"

kill "$cloudxr_pid" 2>/dev/null || true
wait "$cloudxr_pid" 2>/dev/null || true

exit "$status"
