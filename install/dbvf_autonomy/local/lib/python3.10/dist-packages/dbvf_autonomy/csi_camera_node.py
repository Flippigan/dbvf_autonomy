"""CSI camera node — publishes IMX219 images via GStreamer on Jetson Orin Nano."""
import yaml

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def build_gstreamer_pipeline(sensor_id, width, height, framerate):
    """Build GStreamer pipeline string for nvarguscamerasrc."""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},'
        f'framerate={framerate}/1 ! '
        f'nvvidconv ! video/x-raw,format=BGRx ! '
        f'videoconvert ! video/x-raw,format=BGR ! appsink'
    )


def load_camera_info_yaml(yaml_path):
    """Load a ROS camera calibration YAML and return a CameraInfo message."""
    with open(yaml_path, 'r') as f:
        cal = yaml.safe_load(f)

    info = CameraInfo()
    info.width = cal['image_width']
    info.height = cal['image_height']
    info.distortion_model = cal.get('distortion_model', 'plumb_bob')
    info.d = cal['distortion_coefficients']['data']
    info.k = cal['camera_matrix']['data']
    info.r = cal['rectification_matrix']['data']
    info.p = cal['projection_matrix']['data']
    return info


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera')

        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('camera_info_url', '')

        self.sensor_id = self.get_parameter('sensor_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.camera_info_url = self.get_parameter('camera_info_url').value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)

        # Load camera calibration
        self.camera_info_msg = None
        if self.camera_info_url:
            path = self.camera_info_url
            if path.startswith('file://'):
                path = path[len('file://'):]
            try:
                self.camera_info_msg = load_camera_info_yaml(path)
                self.get_logger().info(f'Loaded calibration: {path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load calibration: {e}')

        # Open GStreamer pipeline
        pipeline = build_gstreamer_pipeline(
            self.sensor_id, self.width, self.height, self.framerate)
        self.get_logger().info(f'Opening CSI camera: {pipeline}')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open CSI camera')
            return

        # Timer at target framerate
        period = 1.0 / self.framerate
        self.timer = self.create_timer(period, self._capture_cb)

    def _capture_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera'
        self.image_pub.publish(img_msg)

        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = stamp
            self.camera_info_msg.header.frame_id = 'camera'
            self.info_pub.publish(self.camera_info_msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.get_logger().info('CSI camera released')
        super().destroy_node()


def main():
    rclpy.init()
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
