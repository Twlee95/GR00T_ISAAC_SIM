#!/usr/bin/env bash
set -euo pipefail

pid=$(pgrep -f "[s]onic_xr_pipeline/sonic_xr_teleop.py" | head -1 || true)

if [[ -z "$pid" ]]; then
    echo "Isaac Teleop이 실행 중이 아닙니다."
    exit 0
fi

echo "Isaac Teleop PID=$pid 안전 종료 요청"
kill -INT "$pid"

# HDF5 flush와 환경 종료를 최대 60초 기다린다.
for _ in $(seq 1 240); do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "HDF5 저장 및 Isaac Teleop 종료 완료"
        exit 0
    fi
    sleep 0.25
done

echo "아직 저장 종료 중입니다. 강제 종료하지 마세요."
exit 1
