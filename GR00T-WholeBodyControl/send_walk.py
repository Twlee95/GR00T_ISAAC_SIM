import sys, time
sys.path.insert(0, "/workspace/wbc/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/tests")
from test_zmq_manager import ZMQPublisher

pub = ZMQPublisher(host="*", port=5556, verbose=True)
time.sleep(1.0)

# 제어 시작 (planner 모드)
pub.send_command(start=True, stop=False, planner=True)
time.sleep(1.0)
# 제어 안정화: IDLE로 3초 세우기
print("Idle stabilize...")
for i in range(10):
    pub.send_planner(mode=0, movement=[0.0,0.0,0.0], facing=[1.0,0.0,0.0], speed=-1.0, height=-1.0)
    time.sleep(0.1)

# 전진 걷기 (mode=2 WALK, movement=[1,0,0])
print("Walking forward...")
for i in range(100):  # 10초 전진
    pub.send_planner(mode=2, movement=[1.0,0.0,0.0], facing=[1.0,0.0,0.0], speed=-1.0, height=-1.0)
    time.sleep(0.1)
print("Turning left and walking...")
import math
for i in range(100):  # 10초: 앞->왼쪽으로 점진 좌회전하며 전진
    ang = math.radians(min(90, i*2))  # 0->90도 서서히
    fx, fy = math.cos(ang), math.sin(ang)
    pub.send_planner(mode=2, movement=[fx,fy,0.0], facing=[fx,fy,0.0], speed=-1.0, height=-1.0)
    time.sleep(0.1)

# 정지
time.sleep(0.5)
pub.close()
print("DONE")
