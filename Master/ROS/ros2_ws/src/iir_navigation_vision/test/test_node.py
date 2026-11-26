"""Exercise the vision node without camera or NPU hardware.
在没有摄像头或 NPU 硬件时验证视觉节点。
"""

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
from sensor_msgs_py.point_cloud2 import read_points

from iir_navigation_vision.node import VisionNode


class Runtime:
    def __init__(self, result):
        self.result = result
        self.inputs = []
        self.released = False

    def inference(self, *, inputs):
        self.inputs.append(inputs[0].shape)
        return self.result

    def release(self):
        self.released = True


def calibration(tmp_path):
    camera = tmp_path / 'camera.yaml'
    camera.write_text('''image_width: 100
image_height: 100
distortion_model: plumb_bob
camera_matrix:
  rows: 3
  cols: 3
  data: [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
''', encoding='utf-8')
    ground = tmp_path / 'ground.yaml'
    ground.write_text('''image_width: 100
image_height: 100
base_frame: base_link
homography: [0.0, -0.01, 2.0, -0.01, 0.0, 1.0, 0.0, 0.0, 1.0]
''', encoding='utf-8')
    return camera, ground


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    camera, ground = calibration(tmp_path)
    runtime = Runtime([np.zeros((1, 255, size, size), dtype=np.float32)
                       for size in (80, 40, 20)])
    overrides = [Parameter('camera_calibration_file', value=str(camera)),
                 Parameter('ground_calibration_file', value=str(ground)),
                 Parameter('model_file', value='test.rknn')]
    instance = VisionNode(parameter_overrides=overrides,
                          runtime_factory=lambda _: runtime)
    yield instance, runtime
    instance.destroy_node()
    assert runtime.released
    rclpy.shutdown()


def test_camera_frame_produces_stamped_empty_cloud_for_no_detections(node):
    instance, runtime = node
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = instance.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_link'
    msg.data = cv2.imencode('.jpg', np.zeros((100, 100, 3), np.uint8))[1].tobytes()
    instance.on_image(msg)
    assert runtime.inputs == [(640, 640, 3)]
    assert len(captured) == 1
    assert captured[0].header.stamp == msg.header.stamp
    assert captured[0].header.frame_id == 'base_link'
    assert list(read_points(captured[0])) == []


def test_invalid_image_does_not_publish_a_cloud(node):
    instance, runtime = node
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = instance.get_clock().now().to_msg()
    msg.data = b'invalid jpeg'
    instance.on_image(msg)
    assert captured == []
    assert runtime.inputs == []


def test_runtime_failure_does_not_publish_a_cloud(node):
    instance, runtime = node
    runtime.result = None
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = instance.get_clock().now().to_msg()
    msg.data = cv2.imencode('.jpg', np.zeros((100, 100, 3), np.uint8))[1].tobytes()
    instance.on_image(msg)
    assert captured == []


def test_detected_object_produces_ground_points(node):
    instance, runtime = node
    runtime.result[0][0, :4, 20, 30] = 0.5
    runtime.result[0][0, 4, 20, 30] = 0.9
    runtime.result[0][0, 5, 20, 30] = 0.8
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = instance.get_clock().now().to_msg()
    msg.data = cv2.imencode('.jpg', np.zeros((100, 100, 3), np.uint8))[1].tobytes()
    instance.on_image(msg)
    assert len(captured) == 1
    points = np.asarray(list(read_points(captured[0]).tolist()))
    np.testing.assert_allclose(points, [(1.73359375, 0.6265625, 0.0),
                                        (1.73359375, 0.61875, 0.0),
                                        (1.73359375, 0.6109375, 0.0)], rtol=1e-5)


def test_stale_camera_frame_is_not_processed(node):
    instance, runtime = node
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = (instance.get_clock().now() - Duration(seconds=1.0)).to_msg()
    instance.on_image(msg)
    assert captured == []
    assert runtime.inputs == []


def test_frame_age_includes_processing_time(node, monkeypatch):
    instance, _runtime = node
    captured = []
    instance.publish_cloud = captured.append
    msg = CompressedImage()
    msg.header.stamp = (instance.get_clock().now() - Duration(seconds=0.3)).to_msg()
    msg.data = cv2.imencode('.jpg', np.zeros((100, 100, 3), np.uint8))[1].tobytes()
    clock = iter([0.0, 0.25])
    monkeypatch.setattr('iir_navigation_vision.node.time.monotonic', lambda: next(clock))
    instance.on_image(msg)
    assert captured == []
