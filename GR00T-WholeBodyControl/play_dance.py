import sys, time
sys.path.insert(0, "/workspace/wbc/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/tests")
from test_zmq_manager import ZMQPublisher
pub = ZMQPublisher(host="*", port=5556, verbose=False)
time.sleep(1.0)
pub.send_command(start=True, stop=False, planner=True)
time.sleep(1.0)
print("stabilizing")
for i in range(60):
    pub.send_planner(mode=0, movement=[0,0,0], facing=[1,0,0], speed=-1.0, height=-1.0)
    time.sleep(0.1)
print("STABILIZED")
pub.close()
