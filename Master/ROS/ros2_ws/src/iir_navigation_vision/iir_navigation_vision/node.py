"""Publish calibrated ground obstacles from the shared monocular camera.
从共享单目相机发布经过标定的地面障碍物。
"""

from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import yaml

from .geometry import GroundProjection
from .yolov5 import decode_image, decode_outputs, prepare_image, restore_boxes


def load_projection(camera_path, ground_path):
    """Load measured camera intrinsics and the image-to-ground homography.
    读取实测相机内参和图像到地面的单应矩阵。
    """
    with Path(camera_path).open(encoding='utf-8') as stream:
        camera = yaml.safe_load(stream)
    with Path(ground_path).open(encoding='utf-8') as stream:
        ground = yaml.safe_load(stream)
    if camera['distortion_model'] not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported camera distortion model')
    if (camera['image_width'], camera['image_height']) != (
            ground['image_width'], ground['image_height']):
        raise ValueError('Camera and ground calibration sizes differ')
    if ground['base_frame'] != 'base_link':
        raise ValueError('Ground homography must use the base_link frame')
    return GroundProjection(camera['camera_matrix']['data'],
                            camera['distortion_coefficients']['data'],
                            ground['homography'],
                            width=camera['image_width'],
                            height=camera['image_height'])


def load_runtime(model_path):
    """Load the RK3588 model and initialize the NPU runtime.
    加载 RK3588 模型并初始化 NPU 运行时。
    """
    from rknnlite.api import RKNNLite

    runtime = RKNNLite()
    status = runtime.load_rknn(str(model_path))
    if status != 0:
        runtime.release()
        raise RuntimeError(f'RKNN model load failed: {status}')
    status = runtime.init_runtime()
    if status != 0:
        runtime.release()
        raise RuntimeError(f'RKNN runtime initialization failed: {status}')
    return runtime


class VisionNode(Node):
    """Convert fresh shared-camera frames to ground obstacle point clouds.
    将共享相机的新鲜图像转换为地面障碍物点云。
    """

    def __init__(self, *, parameter_overrides=None, runtime_factory=load_runtime):
        super().__init__('iir_navigation_vision', parameter_overrides=parameter_overrides)
        self.declare_parameter('model_file', '')
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('ground_calibration_file', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('obstacle_classes', [0, 1, 2, 3, 5, 7])
        self.declare_parameter('max_image_age_s', 0.5)

        model = self.get_parameter('model_file').value
        camera = self.get_parameter('camera_calibration_file').value
        ground = self.get_parameter('ground_calibration_file').value
        self.confidence = self.get_parameter('confidence_threshold').value
        self.nms_iou = self.get_parameter('nms_iou_threshold').value
        self.classes = self.get_parameter('obstacle_classes').value
        self.max_image_age = self.get_parameter('max_image_age_s').value
        if not model or not camera or not ground:
            raise ValueError('RKNN model and both calibration files are required')
        if not (0.0 < self.confidence <= 1.0 and 0.0 < self.nms_iou <= 1.0
                and self.max_image_age > 0.0 and self.classes
                and all(0 <= value < 80 for value in self.classes)):
            raise ValueError('Invalid vision configuration')
        self.projector = load_projection(camera, ground)
        self.publisher = self.create_publisher(PointCloud2, '/vision/ground_obstacles',
                                               qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.on_image,
            qos_profile_sensor_data)
        self.runtime = runtime_factory(model)

    def publish_cloud(self, cloud):
        self.publisher.publish(cloud)

    def on_image(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0
        if stamp_ns == 0 or age < 0.0 or age > self.max_image_age:
            self.get_logger().error('Camera frame timestamp is missing or stale')
            return
        started = time.monotonic()
        try:
            image = decode_image(msg.data)
            height, width = image.shape[:2]
            prepared, scale, left, top = prepare_image(image)
            outputs = self.runtime.inference(inputs=[prepared])
            if outputs is None:
                raise RuntimeError('RKNN inference returned no outputs')
            boxes = decode_outputs(outputs, self.confidence, self.nms_iou, self.classes)
            boxes = restore_boxes(boxes, scale, left, top, width, height)
            points = self.projector.project(boxes, width, height)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(f'Vision processing failed: {exc}')
            return
        if age + time.monotonic() - started > self.max_image_age:
            self.get_logger().error('Vision inference exceeded the frame age limit')
            return
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_link'
        self.publish_cloud(create_cloud_xyz32(header, points))

    def destroy_node(self):
        self.runtime.release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
