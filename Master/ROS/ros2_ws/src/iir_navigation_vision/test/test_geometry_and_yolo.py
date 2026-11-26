"""Verify the calibrated projection and RKNN YOLOv5 output contract.
验证标定投影和 RKNN YOLOv5 输出契约。
"""

import numpy as np
import pytest

from iir_navigation_vision.geometry import GroundProjection
from iir_navigation_vision.yolov5 import (decode_image, decode_outputs,
                                        prepare_image, restore_boxes)


def test_letterbox_preserves_original_pixel_coordinates():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    prepared, scale, left, top = prepare_image(image)
    assert prepared.shape == (640, 640, 3)
    assert (scale, left, top) == (0.5, 0, 140)


def test_yolov5_decodes_object_score_and_box():
    outputs = [np.zeros((1, 255, size, size), dtype=np.float32)
               for size in (80, 40, 20)]
    outputs[0][0, :4, 20, 30] = 0.5
    outputs[0][0, 4, 20, 30] = 0.9
    outputs[0][0, 5, 20, 30] = 0.8
    boxes = decode_outputs(outputs, 0.25, 0.45, [0])
    assert len(boxes) == 1
    assert boxes[0] == pytest.approx((239.0, 157.5, 249.0, 170.5))


def test_yolov5_ignores_classes_outside_the_configured_set():
    outputs = [np.zeros((1, 255, size, size), dtype=np.float32)
               for size in (80, 40, 20)]
    outputs[0][0, :4, 20, 30] = 0.5
    outputs[0][0, 4, 20, 30] = 0.9
    outputs[0][0, 6, 20, 30] = 0.8
    assert decode_outputs(outputs, 0.25, 0.45, [0]) == []


def test_yolov5_rejects_a_different_model_output_shape():
    with pytest.raises(ValueError, match='YOLOv5'):
        decode_outputs([np.zeros((1, 84, 8400))], 0.25, 0.45, [0])


def test_boxes_return_from_letterbox_to_original_pixels():
    boxes = restore_boxes([(100.0, 190.0, 200.0, 240.0)], 0.5, 0, 140,
                          1280, 720)
    np.testing.assert_allclose(boxes, [(200.0, 100.0, 400.0, 200.0)])


def test_invalid_compressed_frame_raises_an_error():
    with pytest.raises(ValueError, match='compressed camera frame'):
        decode_image(b'not an image')


def test_ground_projection_uses_undistorted_bottom_edge_and_base_frame():
    camera_matrix = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 50.0],
                              [0.0, 0.0, 1.0]])
    homography = np.array([[0.0, -0.01, 2.0], [-0.01, 0.0, 1.0],
                            [0.0, 0.0, 1.0]])
    projector = GroundProjection(camera_matrix, np.zeros(5), homography,
                                 width=100, height=100)
    points = projector.project([(20.0, 10.0, 40.0, 50.0)], 100, 100)
    np.testing.assert_allclose(points, [(1.5, 0.8, 0.0),
                                        (1.5, 0.7, 0.0),
                                        (1.5, 0.6, 0.0)])


def test_ground_projection_rejects_a_different_image_size():
    projector = GroundProjection(np.eye(3), np.zeros(5), np.eye(3),
                                 width=100, height=100)
    with pytest.raises(ValueError, match='image size'):
        projector.project([], 200, 100)
