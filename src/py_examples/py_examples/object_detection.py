#!/usr/bin/env python3
import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision as mp_vision


def qos_best_effort(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_reliable(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def ros_image_to_bgr8(msg: Image) -> np.ndarray:
    """
    Minimal ROS2 Image -> OpenCV BGR uint8 without cv_bridge.
    Supports: bgr8, rgb8, bgra8, rgba8, mono8
    """
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)

    if enc in ("bgr8", "rgb8"):
        row_step = int(msg.step)
        if row_step < w * 3:
            raise ValueError(f"Invalid step={row_step} for encoding={msg.encoding} width={w}")
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape((h, row_step))[:, : w * 3].reshape((h, w, 3))
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img.copy()

    if enc in ("bgra8", "rgba8"):
        row_step = int(msg.step)
        if row_step < w * 4:
            raise ValueError(f"Invalid step={row_step} for encoding={msg.encoding} width={w}")
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape((h, row_step))[:, : w * 4].reshape((h, w, 4))
        if enc == "rgba8":
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img.copy()

    if enc in ("mono8", "8uc1"):
        row_step = int(msg.step)
        if row_step < w:
            raise ValueError(f"Invalid step={row_step} for encoding={msg.encoding} width={w}")
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        gray = buf.reshape((h, row_step))[:, :w].reshape((h, w))
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return bgr.copy()

    raise NotImplementedError(f"Unsupported Image encoding: '{msg.encoding}'")


def bgr8_to_ros_image(img_bgr: np.ndarray, header) -> Image:
    """
    OpenCV BGR uint8 -> ROS2 sensor_msgs/Image without cv_bridge.
    """
    if img_bgr.dtype != np.uint8:
        raise ValueError(f"Expected uint8 image, got {img_bgr.dtype}")
    if img_bgr.ndim != 3 or img_bgr.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 BGR image, got shape={img_bgr.shape}")

    h, w = img_bgr.shape[:2]
    msg = Image()
    msg.header = header
    msg.height = int(h)
    msg.width = int(w)
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(w * 3)
    msg.data = img_bgr.tobytes()
    return msg


class MpHandsFisheyeNoCvBridge(Node):
    """
    MediaPipe Hands via Tasks API, with fisheye undistort (same style as your YOLO node)
    """

    def __init__(self):
        super().__init__("mp_hands_fisheye_no_cv_bridge")

        # -------- topics --------
        self.declare_parameter("image_topic", "/aima/hal/sensor/stereo_head_front_left/rgb_image/raw_modified")
        self.declare_parameter("annotated_topic", "/gesture/annotated")
        self.declare_parameter("label_topic", "/gesture/label")

        # -------- mediapipe task --------
        self.declare_parameter("hand_task_model", "/home/baymax/models/hand_landmarker.task")
        self.declare_parameter("max_num_hands", 1)
        self.declare_parameter("min_det_conf", 0.5)
        self.declare_parameter("min_track_conf", 0.5)

        # -------- speed --------
        self.declare_parameter("resize_width", 488)   # 0=disable, same spirit as YOLO
        self.declare_parameter("every_n", 1)

        # -------- fisheye (same as YOLO) --------
        self.declare_parameter("use_fisheye_undistort", True)
        self.declare_parameter("balance", 0.0)

        # -------- draw / log --------
        self.declare_parameter("draw_points", True)
        self.declare_parameter("draw_connections", False)  # tasks下画连线需要自定义，这里先关
        self.declare_parameter("line_thickness", 2)
        self.declare_parameter("log_every_n", 30)
        self.declare_parameter("ema_alpha", 0.10)

        # -------- QoS --------
        self.declare_parameter("sub_depth", 5)
        self.declare_parameter("pub_depth", 5)

        # ---- read params
        self.image_topic = self.get_parameter("image_topic").value
        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.label_topic = self.get_parameter("label_topic").value

        self.model_path = self.get_parameter("hand_task_model").value
        self.max_num_hands = int(self.get_parameter("max_num_hands").value)
        self.min_det_conf = float(self.get_parameter("min_det_conf").value)
        self.min_track_conf = float(self.get_parameter("min_track_conf").value)

        self.resize_width = int(self.get_parameter("resize_width").value)
        self.every_n = int(self.get_parameter("every_n").value)
        if self.every_n < 1:
            self.every_n = 1

        self.use_fisheye_undistort = bool(self.get_parameter("use_fisheye_undistort").value)
        self.balance = float(self.get_parameter("balance").value)

        self.draw_points = bool(self.get_parameter("draw_points").value)
        self.draw_connections = bool(self.get_parameter("draw_connections").value)
        self.line_thickness = int(self.get_parameter("line_thickness").value)
        if self.line_thickness < 1:
            self.line_thickness = 1

        self.log_every_n = int(self.get_parameter("log_every_n").value)
        if self.log_every_n < 1:
            self.log_every_n = 1
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        if not (0.0 < self.ema_alpha <= 1.0):
            self.ema_alpha = 0.10

        self.sub_depth = int(self.get_parameter("sub_depth").value)
        self.pub_depth = int(self.get_parameter("pub_depth").value)

        # ---- fixed camera intrinsics (your CameraInfo)
        self.calib_w = 2064
        self.calib_h = 1552
        self.K0 = np.array([
            [692.3122818092, 0.0,            1029.2969725256],
            [0.0,            692.7244199861,  769.1340697814],
            [0.0,            0.0,               1.0],
        ], dtype=np.float64)
        self.D0 = np.array([0.0959434972, -0.0142475411, -0.0050136465, 0.0013407289],
                           dtype=np.float64).reshape(-1, 1)

        # ---- undistort map cache (same as YOLO)
        self._map_cached = False
        self._map_size = None
        self._map1 = None
        self._map2 = None

        # ---- mediapipe tasks landmarker
        base_opts = mp_tasks.BaseOptions(model_asset_path=self.model_path)
        opts = mp_vision.HandLandmarkerOptions(
            base_options=base_opts,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=self.max_num_hands,
            min_hand_detection_confidence=self.min_det_conf,
            min_hand_presence_confidence=self.min_det_conf,
            min_tracking_confidence=self.min_track_conf,
        )
        self.landmarker = mp_vision.HandLandmarker.create_from_options(opts)

        # ---- ROS io
        self.sub = self.create_subscription(
            Image, self.image_topic, self.cb, qos_best_effort(depth=self.sub_depth)
        )
        self.pub_img = self.create_publisher(
            Image, self.annotated_topic, qos_reliable(depth=self.pub_depth)
        )
        self.pub_label = self.create_publisher(
            String, self.label_topic, qos_reliable(depth=self.pub_depth)
        )

        # ---- runtime state / profiling
        self._frame_count = 0
        self.last_label = "NONE"
        self._ema = {k: None for k in ["decode","resize","undist","mp","draw","publish","total"]}

        self.get_logger().info("✅ MpHandsFisheyeNoCvBridge started.")
        self.get_logger().info(f"image={self.image_topic} annotated={self.annotated_topic} label={self.label_topic}")
        self.get_logger().info(f"task_model={self.model_path} max_num_hands={self.max_num_hands}")
        self.get_logger().info(f"resize_width={self.resize_width} every_n={self.every_n}")
        self.get_logger().info(f"fisheye_undistort={self.use_fisheye_undistort} balance={self.balance}")

    def destroy_node(self):
        try:
            if hasattr(self, "landmarker") and self.landmarker is not None:
                self.landmarker.close()
        except Exception:
            pass
        super().destroy_node()

    @staticmethod
    def _ms(t0, t1) -> float:
        return (t1 - t0) * 1000.0

    def _ema_update(self, k, v):
        cur = self._ema[k]
        if cur is None:
            self._ema[k] = v
        else:
            a = self.ema_alpha
            self._ema[k] = (1 - a) * cur + a * v

    def _resize_keep_aspect(self, img_bgr):
        if self.resize_width <= 0:
            h, w = img_bgr.shape[:2]
            return img_bgr, (w, h)
        h, w = img_bgr.shape[:2]
        if w == self.resize_width:
            return img_bgr, (w, h)
        s = self.resize_width / float(w)
        nh = int(round(h * s))
        out = cv2.resize(img_bgr, (self.resize_width, nh), interpolation=cv2.INTER_LINEAR)
        return out, (self.resize_width, nh)

    # -------- fisheye undistort: copy your YOLO style --------
    def _build_fisheye_map_for_size(self, w, h):
        sx = w / float(self.calib_w)
        sy = h / float(self.calib_h)

        K = self.K0.copy()
        K[0, 0] *= sx
        K[1, 1] *= sy
        K[0, 2] *= sx
        K[1, 2] *= sy

        img_size = (w, h)
        newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, self.D0, img_size, np.eye(3), balance=self.balance
        )
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, self.D0, np.eye(3), newK, img_size, cv2.CV_16SC2
        )
        self._map1, self._map2 = map1, map2
        self._map_size = (w, h)
        self._map_cached = True
        self.get_logger().info(f"✅ Cached fisheye map for size={self._map_size} (sx={sx:.3f}, sy={sy:.3f})")

    def _maybe_undistort(self, img_bgr):
        if not self.use_fisheye_undistort:
            return img_bgr
        h, w = img_bgr.shape[:2]
        if (not self._map_cached) or (self._map_size != (w, h)):
            self._build_fisheye_map_for_size(w, h)
        return cv2.remap(img_bgr, self._map1, self._map2, interpolation=cv2.INTER_LINEAR)

    # -------- gesture classification (same rule-set you used before) --------
    def _classify_gesture(self, lm, img_w, img_h):
        """
        lm: list of 21 NormalizedLandmark from MediaPipe Tasks
        """
        pts = [(p.x * img_w, p.y * img_h) for p in lm]

        WRIST = 0
        THUMB_TIP = 4
        INDEX_TIP = 8
        MIDDLE_TIP = 12
        RING_TIP = 16
        PINKY_TIP = 20

        INDEX_PIP = 6
        MIDDLE_PIP = 10
        RING_PIP = 14
        PINKY_PIP = 18

        wrist = pts[WRIST]

        def finger_extended(tip_i, pip_i):
            return _dist(pts[tip_i], wrist) > _dist(pts[pip_i], wrist) * 1.05

        index_ext = finger_extended(INDEX_TIP, INDEX_PIP)
        middle_ext = finger_extended(MIDDLE_TIP, MIDDLE_PIP)
        ring_ext = finger_extended(RING_TIP, RING_PIP)
        pinky_ext = finger_extended(PINKY_TIP, PINKY_PIP)
        ext_count = int(index_ext) + int(middle_ext) + int(ring_ext) + int(pinky_ext)

        ok_dist = _dist(pts[THUMB_TIP], pts[INDEX_TIP])
        palm_scale = _dist(pts[INDEX_PIP], pts[PINKY_PIP]) + 1e-6
        ok_close = (ok_dist / palm_scale) < 0.35

        if ok_close and middle_ext and ring_ext and pinky_ext:
            return "OK"
        if ext_count >= 3:
            return "OPEN_PALM"
        if ext_count == 0:
            return "FIST"
        if index_ext and (not middle_ext) and (not ring_ext) and (not pinky_ext):
            return "ONE"
        if index_ext and middle_ext and (not ring_ext) and (not pinky_ext):
            return "TWO"
        return "UNKNOWN"

    def _draw_points(self, img_bgr, lm):
        h, w = img_bgr.shape[:2]
        for p in lm:
            cx, cy = int(p.x * w), int(p.y * h)
            if 0 <= cx < w and 0 <= cy < h:
                cv2.circle(img_bgr, (cx, cy), 3, (0, 255, 0), -1)

    def cb(self, msg_img: Image):
        self._frame_count += 1
        if (self._frame_count % self.every_n) != 0:
            return

        t_total0 = time.perf_counter()

        # 1) decode
        t0 = time.perf_counter()
        try:
            img_bgr = ros_image_to_bgr8(msg_img)
        except Exception as e:
            self.get_logger().error(f"Image decode failed (no cv_bridge): {e}")
            return
        t1 = time.perf_counter()
        ms_dec = self._ms(t0, t1)

        # 2) resize (same order as your YOLO node)
        t0 = time.perf_counter()
        img_bgr, (rw, rh) = self._resize_keep_aspect(img_bgr)
        t1 = time.perf_counter()
        ms_res = self._ms(t0, t1)

        # 3) undistort
        t0 = time.perf_counter()
        try:
            img_bgr = self._maybe_undistort(img_bgr)
        except Exception as e:
            self.get_logger().warn(f"undistort failed; skip. err={e}")
        t1 = time.perf_counter()
        ms_und = self._ms(t0, t1)

        # 4) mediapipe
        t0 = time.perf_counter()
        label = self.last_label
        hand_lm = None
        try:
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp_vision.MpImage(image_format=mp_vision.ImageFormat.SRGB, data=img_rgb)
            res = self.landmarker.detect(mp_image)

            label = "NONE"
            if res.hand_landmarks and len(res.hand_landmarks) > 0:
                hand_lm = res.hand_landmarks[0]  # 21 points
                h, w = img_bgr.shape[:2]
                label = self._classify_gesture(hand_lm, w, h)

            self.last_label = label
        except Exception as e:
            self.get_logger().warn(f"mediapipe detect failed: {e}")
            label = self.last_label
        t1 = time.perf_counter()
        ms_mp = self._ms(t0, t1)

        # 5) draw
        t0 = time.perf_counter()
        cv2.putText(img_bgr, f"Gesture: {label}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        if self.draw_points and (hand_lm is not None):
            self._draw_points(img_bgr, hand_lm)
        t1 = time.perf_counter()
        ms_draw = self._ms(t0, t1)

        # 6) publish
        t0 = time.perf_counter()
        try:
            out_msg = bgr8_to_ros_image(img_bgr, msg_img.header)
            self.pub_img.publish(out_msg)
            self.pub_label.publish(String(data=label))
        except Exception as e:
            self.get_logger().error(f"Publish failed (no cv_bridge): {e}")
            return
        t1 = time.perf_counter()
        ms_pub = self._ms(t0, t1)

        ms_tot = self._ms(t_total0, time.perf_counter())

        for k, v in [("decode", ms_dec), ("resize", ms_res), ("undist", ms_und),
                     ("mp", ms_mp), ("draw", ms_draw), ("publish", ms_pub), ("total", ms_tot)]:
            self._ema_update(k, v)

        if (self._frame_count % self.log_every_n) == 0:
            ema = self._ema
            fps_inst = 1000.0 / ms_tot if ms_tot > 1e-9 else 0.0
            fps_ema = 1000.0 / ema["total"] if ema["total"] and ema["total"] > 1e-9 else 0.0
            self.get_logger().info(
                f"[timing] resized={rw}x{rh} | "
                f"inst(ms): dec={ms_dec:.1f} res={ms_res:.1f} und={ms_und:.1f} mp={ms_mp:.1f} "
                f"draw={ms_draw:.1f} pub={ms_pub:.1f} tot={ms_tot:.1f} | fps={fps_inst:.1f} | "
                f"ema(ms): dec={ema['decode']:.1f} res={ema['resize']:.1f} und={ema['undist']:.1f} mp={ema['mp']:.1f} "
                f"draw={ema['draw']:.1f} pub={ema['publish']:.1f} tot={ema['total']:.1f} | fps_ema={fps_ema:.1f}"
            )


def main():
    rclpy.init()
    node = MpHandsFisheyeNoCvBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
