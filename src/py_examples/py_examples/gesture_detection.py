#!/usr/bin/env python3
import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String

import mediapipe as mp


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rosimg_to_cv2(msg: Image) -> np.ndarray:
    """
    Convert ROS2 sensor_msgs/Image to OpenCV image WITHOUT cv_bridge.
    Returns image in BGR (uint8).
    Supports: bgr8, rgb8, bgra8, rgba8, mono8
    """
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)

    if h <= 0 or w <= 0:
        raise ValueError(f"Invalid image size: {w}x{h}")

    step = int(msg.step)
    if step <= 0:
        raise ValueError(f"Invalid step: {step}")

    buf = np.frombuffer(msg.data, dtype=np.uint8)

    def reshape_with_step(channels: int) -> np.ndarray:
        row_bytes = w * channels
        if step < row_bytes:
            raise ValueError(f"step({step}) < row_bytes({row_bytes}) for encoding={enc}")
        need = step * h
        if buf.size < need:
            raise ValueError(f"Buffer too small: {buf.size} < {need}")
        arr = buf[:need].reshape((h, step))
        arr = arr[:, :row_bytes].reshape((h, w, channels))
        return arr

    if enc == "bgr8":
        return reshape_with_step(3)

    if enc == "rgb8":
        img = reshape_with_step(3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if enc == "bgra8":
        img = reshape_with_step(4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    if enc == "rgba8":
        img = reshape_with_step(4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    if enc == "mono8":
        img = reshape_with_step(1).reshape((h, w))
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Fallback best-effort by step
    if step == w * 3 and buf.size >= step * h:
        return buf[: step * h].reshape((h, w, 3))

    if step == w * 4 and buf.size >= step * h:
        img = buf[: step * h].reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    raise NotImplementedError(
        f"Unsupported encoding: '{msg.encoding}' (step={step}, w={w}, h={h})"
    )


def cv2_to_rosimg(img_bgr: np.ndarray, header) -> Image:
    """
    Convert OpenCV BGR image to ROS2 sensor_msgs/Image WITHOUT cv_bridge.
    """
    if img_bgr is None or img_bgr.size == 0:
        raise ValueError("Empty image")

    if img_bgr.dtype != np.uint8:
        img_bgr = img_bgr.astype(np.uint8)

    msg = Image()
    msg.header = header

    if img_bgr.ndim == 2:
        h, w = img_bgr.shape
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = int(w)
        msg.data = img_bgr.tobytes()
        return msg

    h, w, c = img_bgr.shape
    if c != 3:
        raise ValueError(f"Expected 3 channels BGR, got {c}")

    msg.height = int(h)
    msg.width = int(w)
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(w * 3)
    msg.data = img_bgr.tobytes()
    return msg


class GestureHandsNoCvBridge(Node):
    def __init__(self):
        super().__init__("gesture_hands_no_cv_bridge")

        # -------- params --------
        self.declare_parameter("image_topic", "/aima/hal/sensor/stereo_head_front_left/rgb_image/raw_modified")
        self.declare_parameter("annotated_topic", "/gesture/annotated")
        self.declare_parameter("label_topic", "/gesture/label")

        self.declare_parameter("max_width", 640)
        self.declare_parameter("every_n", 2)
        self.declare_parameter("min_det_conf", 0.5)
        self.declare_parameter("min_track_conf", 0.5)
        self.declare_parameter("max_num_hands", 1)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.label_topic = str(self.get_parameter("label_topic").value)

        self.max_width = int(self.get_parameter("max_width").value)
        self.every_n = max(1, int(self.get_parameter("every_n").value))

        min_det_conf = float(self.get_parameter("min_det_conf").value)
        min_track_conf = float(self.get_parameter("min_track_conf").value)
        max_num_hands = int(self.get_parameter("max_num_hands").value)

        # -------- ROS IO --------
        self.sub = self.create_subscription(
            Image, self.image_topic, self.cb_image, qos_profile_sensor_data
        )
        self.pub_img = self.create_publisher(Image, self.annotated_topic, 10)
        self.pub_label = self.create_publisher(String, self.label_topic, 10)

        # -------- MediaPipe sanity check --------
        # If user has a local file named mediapipe.py, mp.solutions may disappear.
        if not hasattr(mp, "solutions"):
            self.get_logger().error(
                "❌ mediapipe.mp has no attribute 'solutions'. "
                "常见原因：当前工程目录存在 mediapipe.py/mediapipe 文件夹遮蔽了官方库，或 mediapipe 安装损坏。"
            )
            self.get_logger().error(f"mediapipe loaded from: {getattr(mp, '__file__', 'UNKNOWN')}")
            raise RuntimeError("MediaPipe solutions unavailable")

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            model_complexity=0,  # CPU 省
            min_detection_confidence=min_det_conf,
            min_tracking_confidence=min_track_conf,
        )

        # -------- runtime state --------
        self.frame_idx = 0
        self.last_label = "NONE"

        # FPS: monotonic + EMA
        self._last_t = time.monotonic()
        self._fps_ema = 0.0
        self._fps_alpha = 0.2

        self.get_logger().info(f"✅ Subscribing: {self.image_topic}")
        self.get_logger().info(f"🖼️  Publishing annotated: {self.annotated_topic}")
        self.get_logger().info(f"🏷️  Publishing label: {self.label_topic}")
        self.get_logger().info(f"⚙️  every_n={self.every_n}, max_width={self.max_width}")

    def destroy_node(self):
        # Ensure mediapipe resources are released
        try:
            if hasattr(self, "hands") and self.hands is not None:
                self.hands.close()
        except Exception:
            pass
        super().destroy_node()

    def _resize_keep_aspect(self, img_bgr: np.ndarray) -> np.ndarray:
        h, w = img_bgr.shape[:2]
        if self.max_width <= 0 or w <= self.max_width:
            return img_bgr
        scale = self.max_width / float(w)
        new_w = self.max_width
        new_h = max(1, int(h * scale))
        return cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)

    def _classify_gesture(self, lm, img_w, img_h):
        """
        轻量规则：OPEN_PALM / FIST / OK / ONE / TWO / UNKNOWN
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
            # tip distance from wrist should exceed pip distance (with slack)
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

    def _update_fps(self):
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 1e-6:
            return self._fps_ema
        fps_inst = 1.0 / dt
        if self._fps_ema <= 0.0:
            self._fps_ema = fps_inst
        else:
            self._fps_ema = (1 - self._fps_alpha) * self._fps_ema + self._fps_alpha * fps_inst
        return self._fps_ema

    def cb_image(self, msg: Image):
        self.frame_idx += 1

        # 1) ROS Image -> cv2(BGR)
        try:
            img_bgr = rosimg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"rosimg_to_cv2 failed: {e} | encoding={msg.encoding}")
            return

        # 2) resize
        img_bgr = self._resize_keep_aspect(img_bgr)

        # 3) infer every_n
        do_infer = (self.frame_idx % self.every_n == 0)

        label = self.last_label
        if do_infer:
            try:
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                img_rgb.flags.writeable = False
                res = self.hands.process(img_rgb)
                img_rgb.flags.writeable = True

                label = "NONE"
                if res.multi_hand_landmarks:
                    hand_lm = res.multi_hand_landmarks[0]
                    h, w = img_bgr.shape[:2]
                    label = self._classify_gesture(hand_lm.landmark, w, h)

                    self.mp_draw.draw_landmarks(
                        img_bgr, hand_lm, self.mp_hands.HAND_CONNECTIONS
                    )

                self.last_label = label
            except Exception as e:
                # degrade gracefully: keep last_label, still publish image
                self.get_logger().warn(f"mediapipe inference failed: {e}")
                label = self.last_label

        # 4) overlay
        fps = self._update_fps()
        cv2.putText(img_bgr, f"Gesture: {label}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img_bgr, f"FPS(EMA): {fps:.1f}  every_n={self.every_n}  maxW={self.max_width}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 5) publish annotated
        try:
            out_img = cv2_to_rosimg(img_bgr, msg.header)
            self.pub_img.publish(out_img)
        except Exception as e:
            self.get_logger().error(f"cv2_to_rosimg failed: {e}")

        # 6) publish label (每帧都发，方便下游同步；你也可以改成 do_infer 时才发)
        self.pub_label.publish(String(data=label))


def main():
    rclpy.init()
    node = GestureHandsNoCvBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
