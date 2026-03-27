"""Tag detector adapter — debounce, dual-tag switching, angle computation."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray

from dbvf_msgs.msg import LandingTargetPose, TagStatus


# ---------------------------------------------------------------------------
# Pure utility classes/functions (tested independently of ROS2)
# ---------------------------------------------------------------------------

class DebounceFilter:
    """Rolling-buffer debounce for tag ID switching."""

    def __init__(self, buffer_size=30, threshold=0.8):
        self.buffer_size = buffer_size
        self.threshold = threshold
        self.buffer: list = []
        self.active_id = None

    def update(self, candidate_id):
        """Push candidate (int or None) and return the current active tag ID."""
        self.buffer.append(candidate_id)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

        if candidate_id is None:
            return self.active_id

        if self.active_id is None:
            self.active_id = candidate_id
            return self.active_id

        if candidate_id == self.active_id:
            return self.active_id

        # Different tag — switch only if it dominates the buffer
        count = sum(1 for t in self.buffer if t == candidate_id)
        if count / len(self.buffer) >= self.threshold:
            self.active_id = candidate_id

        return self.active_id


def compute_angles(u, v, cx, cy, fx, fy):
    """Angular offset (radians) from camera center to pixel (u, v)."""
    angle_x = math.atan2(u - cx, fx)
    angle_y = math.atan2(v - cy, fy)
    return angle_x, angle_y


def select_best_tag(detections_by_id, primary_id, secondary_id):
    """Return (tag_id, detection) for the best detected tag, or (None, None)."""
    if primary_id in detections_by_id:
        return primary_id, detections_by_id[primary_id]
    if secondary_id in detections_by_id:
        return secondary_id, detections_by_id[secondary_id]
    return None, None


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class TagDetectorAdapterNode(Node):
    def __init__(self):
        super().__init__('tag_detector_adapter')

        self.declare_parameter('primary_tag_id', 0)
        self.declare_parameter('secondary_tag_id', 1)
        self.declare_parameter('primary_tag_size', 0.6)
        self.declare_parameter('secondary_tag_size', 0.15)
        self.declare_parameter('debounce_buffer_size', 30)
        self.declare_parameter('debounce_threshold', 0.8)
        self.declare_parameter('detection_topic', '/apriltag/detections')

        self.primary_id = self.get_parameter('primary_tag_id').value
        self.secondary_id = self.get_parameter('secondary_tag_id').value
        self.primary_size = self.get_parameter('primary_tag_size').value
        self.secondary_size = self.get_parameter('secondary_tag_size').value

        self.fx = self.fy = self.cx = self.cy = None
        self.frames_since_last = 0

        self.debounce = DebounceFilter(
            buffer_size=self.get_parameter('debounce_buffer_size').value,
            threshold=self.get_parameter('debounce_threshold').value)

        det_topic = self.get_parameter('detection_topic').value
        self.create_subscription(
            AprilTagDetectionArray, det_topic, self._detection_cb, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)

        self.target_pub = self.create_publisher(
            LandingTargetPose, '/dbvf/landing_target_pose', 10)
        self.status_pub = self.create_publisher(
            TagStatus, '/dbvf/tag_status', 10)

        self.get_logger().info('Tag detector adapter started')

    def _camera_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _detection_cb(self, msg):
        detections_by_id = {det.id: det for det in msg.detections}

        candidate_id, candidate_det = select_best_tag(
            detections_by_id, self.primary_id, self.secondary_id)
        active_id = self.debounce.update(candidate_id)

        status = TagStatus()
        status.header = msg.header

        if active_id is not None and active_id in detections_by_id:
            self.frames_since_last = 0
            status.detected = True
            status.active_tag_id = active_id
            status.frames_since_last = 0
            status.confidence = sum(
                1 for t in self.debounce.buffer if t == active_id
            ) / max(len(self.debounce.buffer), 1)

            det = detections_by_id[active_id]
            self._publish_target(msg.header, det, active_id)
        else:
            self.frames_since_last += 1
            status.detected = False
            status.active_tag_id = active_id if active_id is not None else -1
            status.frames_since_last = self.frames_since_last
            status.confidence = 0.0

        self.status_pub.publish(status)

    def _publish_target(self, header, detection, tag_id):
        if self.fx is None:
            return

        target = LandingTargetPose()
        target.header = header
        target.tag_id = tag_id
        target.tag_size = (self.primary_size if tag_id == self.primary_id
                           else self.secondary_size)

        u = detection.centre.x
        v = detection.centre.y
        target.angle_x, target.angle_y = compute_angles(
            u, v, self.cx, self.cy, self.fx, self.fy)

        # Position from pose estimation (angles-only for now; position
        # requires verifying the camera-to-body frame transform in sim)
        target.position_valid = False

        self.target_pub.publish(target)


def main():
    rclpy.init()
    node = TagDetectorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
