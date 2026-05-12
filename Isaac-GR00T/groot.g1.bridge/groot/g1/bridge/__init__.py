"""
GR00T G1 Bridge Extension for Isaac Sim v5
Isaac Sim ↔ SONIC C++ ↔ GR00T N1.7 파이프라인

통신 구조:
  - 카메라 ZMQ PUB (port 5555) → run_vla_inference.py
  - Isaac Sim state ZMQ PUB (port 5559, isaacstate) → dds_bridge → DDS rt/lowstate → SONIC C++
  - SONIC C++ DDS rt/lowcmd → dds_bridge → ZMQ PUB (port 5558) → Isaac Sim SUB → G1 관절 적용
  - SONIC C++ ZMQ PUB (port 5557, g1_debug) → run_vla_inference.py
"""

import omni.ext
import omni.timeline
import omni.physx
import asyncio
import subprocess
import threading
import zmq
import msgpack
import msgpack_numpy as mnp
import numpy as np
import time

URDF_PATH = "/workspace/gr00t/GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data/g1"
URDF_FILE = "g1_29dof_with_hand.urdf"

DDS_BRIDGE_SCRIPT = "/isaac-sim/extsUser/groot.g1.bridge/dds_bridge.py"

# ZMQ 포트
CAMERA_ZMQ_PORT   = 5555  # PUB: 카메라 → run_vla_inference.py
ISAAC_STATE_PORT  = 5559  # PUB: Isaac Sim state → dds_bridge
LOWCMD_ZMQ_PORT   = 5558  # SUB: dds_bridge LowCmd → 관절 적용

STATE_TOPIC = b"isaacstate"

NUM_BODY_DOF = 29
NUM_HAND_DOF = 7

mnp.patch()


def _pack_state(state_dict: dict, topic: bytes) -> bytes:
    payload = msgpack.packb(state_dict)
    return topic + payload


class GR00TG1BridgeExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[G1 Bridge] v5 시작!")
        self._physx_sub = None
        self._step = 0
        self._robot = None
        self._prim_path = None
        self._urdf_loaded = False
        self._initialized = False
        self._camera_initialized = False
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx = omni.physx.get_physx_interface()

        self._latest_low_cmd = None
        self._low_cmd_lock = threading.Lock()

        # ZMQ 컨텍스트
        self._zmq_ctx = zmq.Context()

        # 카메라 PUB
        self._cam_sock = self._zmq_ctx.socket(zmq.PUB)
        self._cam_sock.bind(f"tcp://*:{CAMERA_ZMQ_PORT}")

        # Isaac Sim state PUB → dds_bridge
        self._state_sock = self._zmq_ctx.socket(zmq.PUB)
        self._state_sock.bind(f"tcp://*:{ISAAC_STATE_PORT}")

        # LowCmd SUB ← dds_bridge
        self._lowcmd_sock = self._zmq_ctx.socket(zmq.SUB)
        self._lowcmd_sock.setsockopt(zmq.SUBSCRIBE, b"lowcmd")
        self._lowcmd_sock.setsockopt(zmq.CONFLATE, 1)
        self._lowcmd_sock.setsockopt(zmq.RCVTIMEO, 0)
        self._lowcmd_sock.connect(f"tcp://localhost:{LOWCMD_ZMQ_PORT}")

        # LowCmd 수신 스레드
        self._lowcmd_thread = threading.Thread(
            target=self._lowcmd_worker, daemon=True
        )
        self._lowcmd_thread.start()

        # dds_bridge subprocess 시작
        self._dds_proc = None
        #self._start_dds_bridge()

        async def delayed_init():
            await asyncio.sleep(8.0)
            self._physx_sub = self._physx.subscribe_physics_step_events(
                self._on_physics_step
            )
            self._timeline.play()
            print("[G1 Bridge] Timeline play!")

        asyncio.ensure_future(delayed_init())

    def _start_dds_bridge(self):
        def _bridge_restart_loop():
            while True:
                try:
                    python_bin = "/isaac-sim/kit/python/bin/python3.11"
                    log_file = open("/tmp/dds_bridge.log", "a")
                    proc = subprocess.Popen(
                        [python_bin, DDS_BRIDGE_SCRIPT],
                        stdout=log_file,
                        stderr=log_file,
                    )
                    print(f"[G1 Bridge] DDS bridge 시작 (PID={proc.pid})")
                    proc.wait()
                    log_file.flush()
                    log_file.close()
                    print(f"[G1 Bridge] DDS bridge 종료 (returncode={proc.returncode}), 5초 후 재시작...")
                    time.sleep(5.0)
                except Exception as e:
                    print(f"[G1 Bridge] DDS bridge 재시작 에러: {e}")
                    import traceback; traceback.print_exc()
                    time.sleep(5.0)
        
        threading.Thread(target=_bridge_restart_loop, daemon=True).start()
    def _lowcmd_worker(self):
        while True:
            try:
                raw = self._lowcmd_sock.recv(zmq.NOBLOCK)
                payload = raw[len(b"lowcmd"):]
                data = msgpack.unpackb(payload, raw=False)
                with self._low_cmd_lock:
                    self._latest_low_cmd = data
            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                print(f"[G1 Bridge] LowCmd recv 에러: {e}")
                time.sleep(0.01)

    def _setup_scene(self):
        try:
            from isaacsim.core.api.world import World
            world = World()
            world.scene.add_default_ground_plane()
            print("[G1 Bridge] 바닥 추가!")
        except Exception as e:
            print(f"[G1 Bridge] 씬 설정 에러: {e}")

    def _setup_camera(self):
        try:
            import omni.usd
            from pxr import UsdGeom, Gf
            from isaacsim.sensors.camera import Camera

            stage = omni.usd.get_context().get_stage()
            cam_path = "/World/g1_29dof_with_hand/torso_link/EgoCamera"
            if not stage.GetPrimAtPath(cam_path).IsValid():
                cam_prim = UsdGeom.Camera.Define(stage, cam_path)
                xf = UsdGeom.Xformable(cam_prim.GetPrim())
                xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.1))
                xf.AddRotateXYZOp().Set(Gf.Vec3f(0, -90, 90))
                cam_prim.CreateFocalLengthAttr(15.0)
                cam_prim.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

            self._cam_ego = Camera(
                prim_path=cam_path,
                resolution=(640, 480),
                frequency=30,
            )
            self._cam_ego.initialize()
            self._camera_initialized = True
            print("[G1 Bridge] ego_view 카메라 초기화 완료!")
        except Exception as e:
            print(f"[G1 Bridge] 카메라 설정 에러: {e}")
            self._camera_initialized = False

    def _publish_camera(self):
        if not self._camera_initialized:
            return
        try:
            rgb = self._cam_ego.get_rgb()
            if rgb is None or rgb.shape[0] == 0:
                return
            img = rgb[:, :, :3].astype(np.uint8)
            msg = {
                "images": {"ego_view": img},
                "timestamps": {"ego_view": time.time()}
            }
            payload = msgpack.packb(msg, default=mnp.encode)
            self._cam_sock.send(b"camera" + payload)
        except Exception as e:
            if self._step % 500 == 0:
                print(f"[G1 Bridge] 카메라 전송 에러: {e}")

    def _publish_state(self):
        """Isaac Sim G1 state → ZMQ PUB → dds_bridge → DDS rt/lowstate"""
        if self._robot is None:
            return
        try:
            joints = self._robot.get_joint_positions()
            vels   = self._robot.get_joint_velocities()
            if joints is None:
                return

            joints = np.array(joints, dtype=np.float64)
            vels   = np.array(vels, dtype=np.float64) if vels is not None else np.zeros_like(joints)

            body_q  = joints[:NUM_BODY_DOF].tolist()
            body_dq = vels[:NUM_BODY_DOF].tolist()
            left_hand_q  = joints[NUM_BODY_DOF:NUM_BODY_DOF + NUM_HAND_DOF].tolist() \
                           if len(joints) > NUM_BODY_DOF else [0.0] * NUM_HAND_DOF
            right_hand_q = joints[NUM_BODY_DOF + NUM_HAND_DOF:NUM_BODY_DOF + NUM_HAND_DOF * 2].tolist() \
                           if len(joints) > NUM_BODY_DOF + NUM_HAND_DOF else [0.0] * NUM_HAND_DOF

            state_dict = {
                "body_q":       body_q,
                "body_dq":      body_dq,
                "left_hand_q":  left_hand_q,
                "right_hand_q": right_hand_q,
                "base_quat":    [0.0, 0.0, 0.0, 1.0],
            }

            raw = _pack_state(state_dict, STATE_TOPIC)
            self._state_sock.send(raw)

        except Exception as e:
            if self._step % 100 == 0:
                print(f"[G1 Bridge] state 전송 에러: {e}")

    def _apply_low_cmd(self):
        with self._low_cmd_lock:
            low_cmd = self._latest_low_cmd
            self._latest_low_cmd = None

        if low_cmd is None or self._robot is None:
            return

        try:
            current_joints = np.array(
                self._robot.get_joint_positions(), dtype=np.float32
            )
            target = current_joints.copy()
            motor_cmds = low_cmd.get("motor_cmd", [])

            for i in range(min(NUM_BODY_DOF, len(motor_cmds), len(target))):
                target[i] = float(motor_cmds[i]["q"])

            self._robot.set_joint_positions(target)

            if self._step % 100 == 0:
                print(f"[G1 Bridge] LowCmd 적용: len(motor_cmds)={len(motor_cmds)}, target[0:10]={np.round(target[:10], 3)}")

        except Exception as e:
            if self._step % 100 == 0:
                print(f"[G1 Bridge] LowCmd 적용 에러: {e}")

    def _on_physics_step(self, dt):
        self._step += 1

        if self._step == 1000 and not self._urdf_loaded:
            try:
                from isaacsim.asset.importer.urdf import _urdf
                urdf_interface = _urdf.acquire_urdf_interface()
                import_config = _urdf.ImportConfig()
                import_config.fix_base = False
                import_config.create_physics_scene = False
                import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
                import_config.default_drive_strength = 1000.0
                import_config.default_position_drive_damping = 50.0

                robot = urdf_interface.parse_urdf(URDF_PATH, URDF_FILE, import_config)
                prim_path = urdf_interface.import_robot(
                    URDF_PATH, URDF_FILE, robot, import_config, ""
                )
                print(f"[G1 Bridge] URDF 로드: {prim_path}")
                self._prim_path = prim_path
                self._urdf_loaded = True
                self._setup_scene()
            except Exception as e:
                print(f"[G1 Bridge] URDF 에러: {e}")
                import traceback; traceback.print_exc()

        if self._step == 1200 and self._urdf_loaded and not self._initialized:
            try:
                from isaacsim.core.prims import SingleArticulation
                self._robot = SingleArticulation(prim_path=self._prim_path)
                self._robot.initialize()
                # G1 root 위치 초기화: 월드 원점 근처, 지면 위
                self._robot.set_world_pose(
                    position=np.array([0.0, 0.0, 0.80], dtype=np.float32),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),  # w, x, y, z
                )
                # G1 초기 관절 자세
                home_joints = np.zeros(self._robot.num_dof, dtype=np.float32)
                # SONIC/정책 초기값에 맞춰 최소한 hip 쪽만 살짝 세팅
                if self._robot.num_dof >= 29:
                    home_joints[0] = -0.312
                    home_joints[1] = 0.0
                    home_joints[2] = 0.0
                self._robot.set_joint_positions(home_joints)
                # 속도 초기화
                try:
                    self._robot.set_joint_velocities(np.zeros(self._robot.num_dof, dtype=np.float32))
                except Exception:
                    pass
                self._initialized = True
                print(f"[G1 Bridge] 초기화 완료! DOF={self._robot.num_dof}")
                print("[G1 Bridge] G1 root pose / home joint 설정 완료!")
                self._setup_camera()
            except Exception as e:
                print(f"[G1 Bridge] 초기화 에러: {e}")
                import traceback; traceback.print_exc()
            return

        if not self._initialized:
            return

        # 매 2스텝: state 전송 (500Hz 물리 → 250Hz state)
        if self._step % 2 == 0:
            self._publish_state()

        # 매 10스텝: 카메라 전송
        if self._step % 10 == 0:
            self._publish_camera()

        # 매 스텝: LowCmd 적용
        self._apply_low_cmd()

        if self._step % 200 == 0:
            print(f"[G1 Bridge] step={self._step}")

    def on_shutdown(self):
        print("[G1 Bridge] 종료")
        self._initialized = False
        if self._physx_sub:
            self._physx_sub.unsubscribe()
        if self._dds_proc:
            self._dds_proc.terminate()
        try:
            self._cam_sock.close()
            self._state_sock.close()
            self._lowcmd_sock.close()
            self._zmq_ctx.term()
        except Exception:
            pass