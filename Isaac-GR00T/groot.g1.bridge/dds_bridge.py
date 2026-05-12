"""
DDS Bridge v3:
1. Isaac Sim ZMQ state (port 5559) → DDS PUB rt/lowstate → SONIC C++
2. DDS SUB rt/lowcmd → ZMQ PUB (port 5558) → Isaac Sim 브릿지
DDS 연결 실패 시 재시도
"""
import sys
import time
import threading
import zmq
import msgpack
import numpy as np

DDS_DOMAIN_ID = 0
DDS_LOW_STATE_TOPIC = "rt/lowstate"
DDS_LOW_CMD_TOPIC   = "rt/lowcmd"

ZMQ_ISAAC_STATE_PORT = 5559  # SUB: Isaac Sim state
ZMQ_LOWCMD_PORT      = 5558  # PUB: LowCmd → Isaac Sim 브릿지

NUM_BODY_MOTOR = 29


def main():
    print("[DDS Bridge v3] 시작!")

    ctx = zmq.Context()

    # Isaac Sim state SUB
    state_sock = ctx.socket(zmq.SUB)
    state_sock.setsockopt(zmq.SUBSCRIBE, b"isaacstate")
    state_sock.setsockopt(zmq.CONFLATE, 1)
    state_sock.setsockopt(zmq.RCVTIMEO, 100)
    state_sock.connect(f"tcp://localhost:{ZMQ_ISAAC_STATE_PORT}")
    print(f"[DDS Bridge v3] Isaac state SUB: port {ZMQ_ISAAC_STATE_PORT}")

    # LowCmd ZMQ PUB
    lowcmd_pub = ctx.socket(zmq.PUB)
    for _ in range(10):
        try:
            lowcmd_pub.bind(f"tcp://*:{ZMQ_LOWCMD_PORT}")
            break
        except Exception:
            time.sleep(1.0)
    time.sleep(0.5)
    print(f"[DDS Bridge v3] LowCmd ZMQ PUB: port {ZMQ_LOWCMD_PORT}")

    # DDS 초기화 - 재시도 루프
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber,
        ChannelPublisher,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowState_ as LowState_default,
    )
    import signal

    def timeout_handler(signum, frame):
        raise TimeoutError("DDS topic creation timeout")

    #ChannelFactoryInitialize(DDS_DOMAIN_ID, "lo")

    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.core import Qos

    config = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="lo" multicast="false"/>
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>'''

    #domain = Domain(DDS_DOMAIN_ID, config)
    participant = DomainParticipant(DDS_DOMAIN_ID)



    print(f"[DDS Bridge v3] DDS 초기화 완료 (domain={DDS_DOMAIN_ID})")



    from cyclonedds.pub import DataWriter, Publisher
    from cyclonedds.sub import DataReader, Subscriber
    from cyclonedds.topic import Topic

    print(f"[DDS Bridge v3] DDS topic 생성 중...")
    topic_lowstate = Topic(participant, DDS_LOW_STATE_TOPIC, LowState_)
    writer = DataWriter(Publisher(participant), topic_lowstate)
    print(f"[DDS Bridge v3] DDS LowState PUB: {DDS_LOW_STATE_TOPIC}")

    topic_lowcmd = Topic(participant, DDS_LOW_CMD_TOPIC, LowCmd_)
    reader = DataReader(Subscriber(participant), topic_lowcmd)
    print(f"[DDS Bridge v3] DDS LowCmd SUB: {DDS_LOW_CMD_TOPIC}")

    def lowcmd_reader_thread():
        while True:
            try:
                samples = reader.take(10)
                if samples:
                    for sample in samples:
                        try:
                            motor_cmds = []
                            for i in range(min(NUM_BODY_MOTOR, len(sample.motor_cmd))):
                                mc = sample.motor_cmd[i]
                                motor_cmds.append({
                                    "q":   float(mc.q),
                                    "dq":  float(mc.dq),
                                    "tau": float(mc.tau),
                                    "kp":  float(mc.kp),
                                    "kd":  float(mc.kd),
                                })
                            payload = msgpack.packb({"motor_cmd": motor_cmds})
                            lowcmd_pub.send(b"lowcmd" + payload)
                        except Exception as e:
                            print(f"[DDS Bridge v3] LowCmd 직렬화 에러: {e}")
                else:
                    time.sleep(0.001)
            except Exception:
                time.sleep(0.001)

    threading.Thread(target=lowcmd_reader_thread, daemon=True).start()




    # 메인 루프: Isaac Sim state → DDS LowState
    low_state = LowState_default()
    print("[DDS Bridge v3] 메인 루프 시작!")

    while True:
        try:
            raw = state_sock.recv()
            payload = raw[len(b"isaacstate"):]
            state = msgpack.unpackb(payload, raw=False)

            body_q  = state.get("body_q")  or state.get(b"body_q")
            body_dq = state.get("body_dq") or state.get(b"body_dq")
            base_quat = state.get("base_quat") or state.get(b"base_quat")

            if body_q is None:
                continue

            body_q  = list(body_q)
            body_dq = list(body_dq) if body_dq else [0.0] * NUM_BODY_MOTOR

            for i in range(min(NUM_BODY_MOTOR, len(body_q))):
                low_state.motor_state[i].q = body_q[i]
                low_state.motor_state[i].dq = body_dq[i] if i < len(body_dq) else 0.0
                low_state.motor_state[i].tau_est = 0.0

            if base_quat and len(base_quat) == 4:
                low_state.imu_state.quaternion[0] = float(base_quat[3])  # w
                low_state.imu_state.quaternion[1] = float(base_quat[0])  # x
                low_state.imu_state.quaternion[2] = float(base_quat[1])  # y
                low_state.imu_state.quaternion[3] = float(base_quat[2])  # z

            writer.write(low_state)

        except zmq.Again:
            time.sleep(0.001)
        except Exception as e:
            print(f"[DDS Bridge v3] 루프 에러: {e}")
            time.sleep(0.01)


if __name__ == "__main__":
    main()