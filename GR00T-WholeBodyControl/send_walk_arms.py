import sys, time, math
sys.path.insert(0, "/workspace/wbc/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/tests")
from test_zmq_manager import ZMQPublisher

pub = ZMQPublisher(host="*", port=5556, verbose=True)
time.sleep(1.0)

pub.send_command(start=True, stop=False, planner=True)
time.sleep(1.0)
print("Idle stabilize...")
for i in range(10):
    pub.send_planner(mode=0, movement=[0.0,0.0,0.0], facing=[1.0,0.0,0.0], speed=-1.0, height=-1.0)
    time.sleep(0.1)

print("Walking forward...")
for i in range(100):  # 10초 전진 (팔은 SONIC 자동)
    pub.send_planner(mode=2, movement=[1.0,0.0,0.0], facing=[1.0,0.0,0.0], speed=-1.0, height=-1.0)
    time.sleep(0.1)

# 걷기 후: 제자리에서 팔 앞으로 뻗고 손 폈다 쥐기
print("Reach arms and open/close hands...")
lh_base = [0.0, 0.0, 1.75, -1.57, -1.75, -1.57, -1.75]
rh_base = [0.0, 0.0, -1.75, 1.57, 1.75, 1.57, 1.75]
for i in range(80):  # 8초
    ub_pos = [0.0]*17
    ub_pos[3] = -0.8   # 왼 어깨 피치 음수 = 팔 앞으로
    ub_pos[4] = -0.8   # 오른 어깨 피치 음수 = 팔 앞으로
    sec = i * 0.1
    open_factor = 1.0 if (sec % 2.0) >= 1.0 else 0.0   # 1초마다 폈다 쥐기
    lh = [v*(1.0-open_factor) for v in lh_base]
    rh = [v*(1.0-open_factor) for v in rh_base]
    pub.send_planner(mode=0, movement=[0.0,0.0,0.0], facing=[1.0,0.0,0.0],
                     speed=-1.0, height=-1.0,
                     upper_body_position=ub_pos, upper_body_velocity=[0.0]*17,
                     left_hand_joints=lh, right_hand_joints=rh)
    time.sleep(0.1)

time.sleep(0.5)
pub.close()
print("DONE")
