"""Real-time SMPL source for Isaac Sim teleoperation.

Wraps an already-constructed MotionLibRobot and overrides only the SMPL joint
reads so they serve the latest PICO frame (ZMQ port 5556, topic 'pose') instead
of a stored motion clip. Every other attribute/method is delegated to the real
instance, so the existing command/observation pipeline is untouched.

Standalone: modifies no existing files.

Wire format (gear_sonic/utils/teleop/zmq/zmq_planner_sender.py):
    [topic bytes][1280-byte JSON header][concatenated little-endian binary fields]
    header = {"v","endian","count","fields":[{"name","dtype","shape"}, ...]}
"""

import json
import threading

import numpy as np
import torch

try:
    import zmq
except ImportError as e:
    raise RuntimeError("pyzmq required for live SMPL source") from e

HEADER_SIZE = 1280
_DTYPE_MAP = {
    "f32": np.float32, "f64": np.float64,
    "i32": np.int32, "i64": np.int64,
    "u8": np.uint8, "bool": np.bool_,
}


def unpack_pose_message(msg: bytes, topic: str = "pose") -> dict:
    """Inverse of pack_pose_message. Returns {field_name: np.ndarray}."""
    topic_bytes = topic.encode("utf-8")
    if msg.startswith(topic_bytes):
        offset = len(topic_bytes)
    else:
        offset = msg.find(b"{")
    header_raw = msg[offset:offset + HEADER_SIZE].rstrip(b"\x00")
    header = json.loads(header_raw.decode("utf-8"))
    binary = msg[offset + HEADER_SIZE:]
    out = {}
    pos = 0
    for f in header["fields"]:
        dt = np.dtype(_DTYPE_MAP[f["dtype"]]).newbyteorder("<")
        shape = tuple(f["shape"])
        n = int(np.prod(shape)) if shape else 1
        nbytes = n * dt.itemsize
        arr = np.frombuffer(binary[pos:pos + nbytes], dtype=dt).reshape(shape)
        out[f["name"]] = arr
        pos += nbytes
    return out


class LiveSmplBuffer:
    """Subscribes to the PICO pose stream; holds the latest decoded frame dict."""

    def __init__(self, zmq_host="localhost", zmq_port=5556, topic="pose"):
        self._topic = topic
        self._lock = threading.Lock()
        self._latest = None
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://{zmq_host}:{zmq_port}")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._sock.setsockopt(zmq.RCVTIMEO, 1000)
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                msg = self._sock.recv()
                data = unpack_pose_message(msg, self._topic)
                with self._lock:
                    self._latest = data
            except zmq.Again:
                continue
            except Exception:
                continue

    def get_latest(self):
        with self._lock:
            return self._latest

    def close(self):
        self._running = False


class LiveSmplMotionLib:
    """Delegates to a real MotionLibRobot except SMPL joint reads, which come live."""

    def __init__(self, real_motion_lib, live_buffer):
        object.__setattr__(self, "_real", real_motion_lib)
        object.__setattr__(self, "_buf", live_buffer)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_real"), name)

    def _latest(self):
        return object.__getattribute__(self, "_buf").get_latest()

    def get_smpl_joints(self, motion_ids, motion_steps):
        real = object.__getattribute__(self, "_real")
        data = self._latest()
        if data is None or "smpl_joints" not in data:
            return real.get_smpl_joints(motion_ids, motion_steps)
        joints = np.asarray(data["smpl_joints"])
        latest = joints[-1] if joints.ndim == 3 else joints  # (24, 3)
        n = motion_steps.shape[0] if hasattr(motion_steps, "shape") else len(motion_ids)
        out = torch.as_tensor(latest, dtype=torch.float32, device=real._device)
        return out.unsqueeze(0).expand(n, *out.shape).contiguous()

    def latest_root_quat(self):
        real = object.__getattribute__(self, "_real")
        data = self._latest()
        if data is None or "body_quat_w" not in data:
            return None
        bq = np.asarray(data["body_quat_w"])
        if bq.ndim == 1:      # (4,)
            root = bq
        elif bq.ndim == 2:    # (frames, 4) -> latest frame
            root = bq[-1]
        else:                 # (frames, joints, 4) -> latest frame, root joint
            root = bq[-1, 0]
        return torch.as_tensor(root, dtype=torch.float32, device=real._device)
