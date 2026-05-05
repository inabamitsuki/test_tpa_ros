#!/usr/bin/env python3
# encoding: utf-8
# ─────────────────────────────────────────────────────────────────────────────
#  yahboom_nav / scripts / cam_pose_apriltag.py
#
#  PATH FIX  (must run before any other import)
#  ─────────────────────────────────────────────
#  ROS Humble ships its own apriltag.cpython-310.so under /opt/ros/humble/
#  which only exposes apriltag.apriltag (lowercase class) and shadows BOTH
#  pip packages (apriltag 0.0.16 and pupil-apriltags) installed under
#  ~/.local/lib/python3.10/site-packages.
#  Moving ~/.local to the front of sys.path forces Python to find
#  pupil-apriltags first, before the ROS system file.
# ─────────────────────────────────────────────────────────────────────────────
import sys
import os

_pip_local = os.path.expanduser('~/.local/lib/python3.10/site-packages')
if _pip_local in sys.path:
    sys.path.remove(_pip_local)
sys.path.insert(0, _pip_local)
# ─────────────────────────────────────────────────────────────────────────────

import threading
import queue
import time

import cv2 as cv
import numpy as np
import mediapipe as mp

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from yahboomcar_msgs.msg import PointArray
from cv_bridge import CvBridge


# ── AprilTag loader – tries every known API variant ──────────────────────────
def _load_apriltag_detector():
    """
    Returns (detector, kind_str).
    Priority: pupil-apriltags > apriltag-0.0.16 > OpenCV ArUco fallback.
    The ROS system apriltag is intentionally skipped (only has lowercase class,
    no proper detector API).
    """
    import importlib

    # Re-import fresh now that sys.path is fixed
    if 'apriltag' in sys.modules:
        del sys.modules['apriltag']

    # ── A: pupil-apriltags ────────────────────────────────────────────────
    try:
        at = importlib.import_module('apriltag')
        if hasattr(at, 'Detector') and at.__file__ and '.local' in at.__file__:
            det = at.Detector(families='tag36h11')
            det.detect(np.zeros((100, 100), np.uint8))   # sanity check
            print(f'[AprilTag] pupil-apriltags ({at.__file__})')
            return det, 'pupil'
    except Exception as e:
        print(f'[AprilTag] pupil-apriltags failed: {e}')

    # ── B: old apriltag 0.0.16 ────────────────────────────────────────────
    try:
        at = importlib.import_module('apriltag')
        if hasattr(at, 'DetectorOptions'):
            det = at.Detector(at.DetectorOptions(families='tag36h11'))
            print(f'[AprilTag] apriltag-0.0.16 ({at.__file__})')
            return det, 'old'
    except Exception as e:
        print(f'[AprilTag] apriltag-0.0.16 failed: {e}')

    # ── C: OpenCV ArUco (always available) ───────────────────────────────
    try:
        d = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
        p = cv.aruco.DetectorParameters()
        print('[AprilTag] WARNING: using OpenCV ArUco fallback')
        print('           For better results: pip install pupil-apriltags')
        return (d, p), 'aruco'
    except Exception as e:
        print(f'[AprilTag] OpenCV ArUco fallback also failed: {e}')

    print('[AprilTag] FATAL: no working detector. Run:')
    print('  pip install pupil-apriltags')
    return None, 'none'


def _detect_tags(detector, kind, gray):
    """Unified detection → list of (tag_id, corners_int_4x2, center_xy)."""
    out = []
    if kind == 'none' or detector is None:
        return out
    try:
        if kind in ('pupil', 'old'):
            for t in detector.detect(gray):
                out.append((t.tag_id, t.corners.astype(int), t.center))
        elif kind == 'aruco':
            d_dict, d_params = detector
            aruco_det = cv.aruco.ArucoDetector(d_dict, d_params)
            corners_list, ids, _ = aruco_det.detectMarkers(gray)
            if ids is not None:
                for c, tid in zip(corners_list, ids.flatten()):
                    pts = c[0].astype(int)
                    out.append((int(tid), pts, pts.mean(axis=0)))
    except Exception as e:
        print(f'[AprilTag] detect error: {e}')
    return out
# ─────────────────────────────────────────────────────────────────────────────


class CamPoseAprilTagNode(Node):

    def __init__(self):
        super().__init__('cam_pose_apriltag_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('camera_topic',    '/espRos/esp32camera')
        self.declare_parameter('mediapipe_topic', '/mediapipe/points')
        self.declare_parameter('raw_image_topic', '/camera/image_raw')
        self.declare_parameter('at_id_topic',     '/vision/latest_at_id')
        self.declare_parameter('show_window',     True)

        cam_topic  = self.get_parameter('camera_topic').value
        mp_topic   = self.get_parameter('mediapipe_topic').value
        raw_topic  = self.get_parameter('raw_image_topic').value
        at_topic   = self.get_parameter('at_id_topic').value
        self._show = self.get_parameter('show_window').value

        # ── AprilTag detector ─────────────────────────────────────────────
        self._at_det, self._at_kind = _load_apriltag_detector()

        # ── MediaPipe ─────────────────────────────────────────────────────
        self._bridge  = CvBridge()
        mp_pose       = mp.solutions.pose
        mp_draw       = mp.solutions.drawing_utils
        self._pose    = mp_pose.Pose(
            static_image_mode=False,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._pose_conns = mp_pose.POSE_CONNECTIONS
        self._lm_spec    = mp_draw.DrawingSpec(color=(0,0,255), thickness=-1, circle_radius=6)
        self._conn_spec  = mp_draw.DrawingSpec(color=(0,255,0), thickness=2,  circle_radius=2)
        self._mp_draw    = mp_draw

        # ── publishers ────────────────────────────────────────────────────
        self._pub_points = self.create_publisher(PointArray, mp_topic,  10)
        self._pub_raw    = self.create_publisher(Image,      raw_topic,  1)
        self._pub_at_id  = self.create_publisher(String,     at_topic,  10)

        # ── camera_info publisher (required by apriltag_ros CameraSubscriber) ─
        self._pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 1)
        # Approximate intrinsics for 640×480 ESP32-CAM (adjust fx/fy/cx/cy
        # after running camera_calibration if you need exact tag distances).
        _fx, _fy, _cx, _cy = 600.0, 600.0, 320.0, 240.0
        self._cam_info = CameraInfo()
        self._cam_info.width  = 640
        self._cam_info.height = 480
        self._cam_info.distortion_model = 'plumb_bob'
        self._cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._cam_info.k = [_fx, 0.0, _cx, 0.0, _fy, _cy, 0.0, 0.0, 1.0]
        self._cam_info.r = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
        self._cam_info.p = [_fx, 0.0, _cx, 0.0,  0.0, _fy, _cy, 0.0,  0.0, 0.0, 1.0, 0.0]

        # ── subscriber  (BEST_EFFORT = matches ESP32-CAM micro-ROS QoS) ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            CompressedImage, cam_topic, self._on_image, qos)

        # ── depth-1 frame queue (always process latest, drop stale) ──────
        self._q_data   = queue.Queue(maxsize=1)
        self._q_header = queue.Queue(maxsize=1)

        # ── state ─────────────────────────────────────────────────────────
        self._latest_at_id = 'Waiting for Tag...'
        self._pose_status  = 'Not Found'
        self._fps          = 0.0
        self._last_ts      = time.time()

        # ── worker thread (all heavy CV off the ROS spin thread) ──────────
        self._running = True
        self._worker  = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'[CamPoseAprilTag] Ready. apriltag backend: {self._at_kind}\n'
            f'  sub  {cam_topic}\n'
            f'  pub  {mp_topic} | {raw_topic} | {at_topic}'
        )

    # ── ROS callback: just enqueue, never block ───────────────────────────
    def _on_image(self, msg: CompressedImage):
        for q in (self._q_data, self._q_header):
            try:
                q.get_nowait()
            except queue.Empty:
                pass
        self._q_data.put(msg.data)
        self._q_header.put(msg.header)

    # ── Worker thread ──────────────────────────────────────────────────────
    def _process_loop(self):
        while self._running:
            try:
                data   = self._q_data.get(timeout=1.0)
                header = self._q_header.get_nowait()
            except queue.Empty:
                continue

            buf   = np.frombuffer(data, np.uint8)
            frame = cv.imdecode(buf, cv.IMREAD_COLOR)
            if frame is None:
                continue
            frame = cv.resize(frame, (640, 480))

            # Re-publish as raw for apriltag_ros
            raw_msg        = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw_msg.header = header
            self._pub_raw.publish(raw_msg)

            # Publish camera_info with identical header so apriltag_ros
            # message_filters::TimeSynchronizer can pair them.
            self._cam_info.header = header
            self._pub_info.publish(self._cam_info)

            # ── AprilTag ──────────────────────────────────────────────────
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            tags = _detect_tags(self._at_det, self._at_kind, gray)
            for (tid, corners, center) in tags:
                for i in range(4):
                    cv.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]),
                            (255, 0, 255), 2)
                cv.putText(frame, f'ID:{tid}',
                           (int(center[0])-20, int(center[1])-10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                self._latest_at_id = f'ID: {tid}'
                msg_id      = String()
                msg_id.data = str(tid)
                self._pub_at_id.publish(msg_id)

            # ── MediaPipe ─────────────────────────────────────────────────
            pa       = PointArray()
            rgb      = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            results  = self._pose.process(rgb)
            detected = results.pose_landmarks is not None
            if detected:
                self._mp_draw.draw_landmarks(
                    frame, results.pose_landmarks,
                    self._pose_conns, self._lm_spec, self._conn_spec)
                for lm in results.pose_landmarks.landmark:
                    pt     = Point()
                    pt.x, pt.y, pt.z = lm.x, lm.y, lm.z
                    pa.points.append(pt)

            self._pub_points.publish(pa)
            self._pose_status = 'Detected' if detected else 'Not Found'

            # ── FPS ───────────────────────────────────────────────────────
            now           = time.time()
            self._fps     = 1.0 / max(now - self._last_ts, 1e-6)
            self._last_ts = now
            cv.putText(frame, f'FPS:{int(self._fps)}',
                       (10, 35), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

            # ── Dashboard window ──────────────────────────────────────────
            if self._show:
                panel = np.zeros((480, 400, 3), np.uint8)
                cv.putText(panel, 'VISION DASHBOARD',
                           (20,50), cv.FONT_HERSHEY_SIMPLEX, 0.85, (255,255,255), 2)
                cv.line(panel, (20,68), (375,68), (255,255,255), 1)

                cv.putText(panel, f'backend: {self._at_kind}',
                           (20,100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (140,200,140), 1)
                cv.putText(panel, 'Latest tag:',
                           (20,140), cv.FONT_HERSHEY_SIMPLEX, 0.55, (180,180,180), 1)
                cv.putText(panel, self._latest_at_id,
                           (20,178), cv.FONT_HERSHEY_SIMPLEX, 0.85, (255,0,255), 2)

                p_col = (0,255,0) if detected else (0,0,255)
                cv.putText(panel, f'Pose: {self._pose_status}',
                           (20,235), cv.FONT_HERSHEY_SIMPLEX, 0.75, p_col, 2)
                cv.putText(panel, f'FPS: {int(self._fps)}',
                           (20,290), cv.FONT_HERSHEY_SIMPLEX, 0.7, (160,200,0), 2)

                for i, t in enumerate(['/mediapipe/points',
                                        '/camera/image_raw',
                                        '/vision/latest_at_id']):
                    cv.putText(panel, t, (20, 370+i*25),
                               cv.FONT_HERSHEY_SIMPLEX, 0.42, (80,200,80), 1)

                cv.imshow('Yahboom Vision', np.hstack((frame, panel)))
                cv.waitKey(1)

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=2.0)
        cv.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    print('Starting Yahboom Vision Node...')
    rclpy.init(args=args)
    node     = CamPoseAprilTagNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
