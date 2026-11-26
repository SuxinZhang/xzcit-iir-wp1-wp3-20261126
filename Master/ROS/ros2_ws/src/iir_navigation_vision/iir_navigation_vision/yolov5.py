"""Preprocess and decode Rockchip's YOLOv5s ReLU RKNN model.
预处理图像并解码 Rockchip YOLOv5s ReLU RKNN 模型。

Output decoding follows the Apache-2.0 Rockchip model-zoo YOLOv5 example:
输出解码遵循 Apache-2.0 许可的 Rockchip 模型库 YOLOv5 示例：
https://github.com/airockchip/rknn_model_zoo/tree/main/examples/yolov5
"""

import cv2
import numpy as np


SIZE = 640
ANCHORS = (
    ((10, 13), (16, 30), (33, 23)),
    ((30, 61), (62, 45), (59, 119)),
    ((116, 90), (156, 198), (373, 326)),
)


def decode_image(data):
    """Decode one compressed camera frame or raise a data error.
    解码一帧压缩图像，数据无效时抛出错误。
    """
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Cannot decode compressed camera frame')
    return image


def prepare_image(image):
    """Letterbox a BGR frame and return RGB input plus inverse-transform values.
    将 BGR 图像等比填充，返回 RGB 输入及逆变换参数。
    """
    height, width = image.shape[:2]
    scale = min(SIZE / width, SIZE / height)
    new_width = round(width * scale)
    new_height = round(height * scale)
    left = (SIZE - new_width) // 2
    top = (SIZE - new_height) // 2
    canvas = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)
    canvas[top:top + new_height, left:left + new_width] = cv2.resize(
        image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), scale, left, top


def decode_outputs(outputs, confidence, nms_iou, obstacle_classes):
    """Decode the three 255-channel model outputs into input-image boxes.
    将模型的三个 255 通道输出解码成输入图像检测框。
    """
    if len(outputs) != 3:
        raise ValueError('Expected three YOLOv5 output tensors')
    boxes = []
    scores = []
    classes = []
    for output, size, anchors in zip(outputs, (80, 40, 20), ANCHORS):
        data = np.asarray(output)
        if data.shape != (1, 255, size, size) or not np.isfinite(data).all():
            raise ValueError('Invalid YOLOv5 output tensor')
        values = data.reshape(3, 85, size, size)
        class_ids = values[:, 5:, :, :].argmax(axis=1)
        class_scores = values[:, 5:, :, :].max(axis=1)
        confidence_values = values[:, 4, :, :] * class_scores
        selected = (confidence_values >= confidence) & np.isin(class_ids, obstacle_classes)
        for anchor, row, col in zip(*np.nonzero(selected)):
            xy = (values[anchor, :2, row, col] * 2.0 - 0.5
                  + np.array([col, row])) * (SIZE / size)
            wh = ((values[anchor, 2:4, row, col] * 2.0) ** 2
                  * np.asarray(anchors[anchor]))
            boxes.append((xy[0] - wh[0] / 2, xy[1] - wh[1] / 2,
                          xy[0] + wh[0] / 2, xy[1] + wh[1] / 2))
            scores.append(float(confidence_values[anchor, row, col]))
            classes.append(int(class_ids[anchor, row, col]))
    if not boxes:
        return []
    boxes = np.asarray(boxes, dtype=np.float64)
    scores = np.asarray(scores)
    classes = np.asarray(classes)
    keep = []
    for class_id in np.unique(classes):
        order = np.flatnonzero(classes == class_id)
        order = order[np.argsort(scores[order])[::-1]]
        while order.size:
            index = order[0]
            keep.append(index)
            remaining = order[1:]
            intersection_min = np.maximum(boxes[index, :2], boxes[remaining, :2])
            intersection_max = np.minimum(boxes[index, 2:], boxes[remaining, 2:])
            intersection = np.maximum(intersection_max - intersection_min, 0).prod(axis=1)
            areas = np.maximum(boxes[:, 2:] - boxes[:, :2], 0).prod(axis=1)
            union = areas[index] + areas[remaining] - intersection
            iou = np.divide(intersection, union, out=np.zeros_like(intersection),
                            where=union > 0)
            order = remaining[iou <= nms_iou]
    return [tuple(float(value) for value in boxes[index]) for index in keep]


def restore_boxes(boxes, scale, left, top, width, height):
    """Restore letterboxed boxes to the original image bounds.
    将等比填充后的检测框还原至原图范围。
    """
    restored = []
    for x1, y1, x2, y2 in boxes:
        x1 = np.clip((x1 - left) / scale, 0.0, width - 1.0)
        x2 = np.clip((x2 - left) / scale, 0.0, width - 1.0)
        y1 = np.clip((y1 - top) / scale, 0.0, height - 1.0)
        y2 = np.clip((y2 - top) / scale, 0.0, height - 1.0)
        if x2 > x1 and y2 > y1:
            restored.append((float(x1), float(y1), float(x2), float(y2)))
    return restored
