"""Project detected ground contact points into the robot frame.
将检测到的地面接触点投影到机器人坐标系。
"""

import cv2
import numpy as np


class GroundProjection:
    """Use camera calibration and a measured image-to-ground homography.
    使用相机标定结果和实测的图像到地面单应矩阵。
    """

    def __init__(self, camera_matrix, distortion, homography, *, width, height):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
        self.homography = np.asarray(homography, dtype=np.float64).reshape(3, 3)
        self.width = int(width)
        self.height = int(height)
        if (self.width <= 0 or self.height <= 0
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.camera_matrix).all()
                or not np.isfinite(self.distortion).all()
                or not np.isfinite(self.homography).all()
                or np.linalg.matrix_rank(self.camera_matrix) != 3
                or np.linalg.matrix_rank(self.homography) != 3):
            raise ValueError('Invalid camera calibration or ground homography')

    def project(self, boxes, width, height):
        """Project the left, centre and right points of each box's bottom edge.
        投影每个检测框底边的左、中、右三个点。
        """
        if (width, height) != (self.width, self.height):
            raise ValueError('Input image size differs from camera calibration')
        if not boxes:
            return []
        pixels = np.asarray([(x, bottom) for left, _, right, bottom in boxes
                             for x in (left, (left + right) / 2.0, right)],
                            dtype=np.float64).reshape(-1, 1, 2)
        undistorted = cv2.undistortPoints(pixels, self.camera_matrix,
                                          self.distortion, P=self.camera_matrix)
        homogeneous = np.column_stack((undistorted.reshape(-1, 2),
                                       np.ones(len(undistorted))))
        projected = homogeneous @ self.homography.T
        if not np.isfinite(projected).all() or np.any(np.abs(projected[:, 2]) < 1e-9):
            raise ValueError('Ground projection produced non-finite coordinates')
        points = projected[:, :2] / projected[:, 2, None]
        return [(float(x), float(y), 0.0) for x, y in points]
