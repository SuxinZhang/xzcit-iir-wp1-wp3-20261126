"""Exercise the configured sensor layers with a real Nav2 costmap.
使用真实 Nav2 代价地图验证配置的传感器图层。
"""

import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, Range
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
import yaml


PARAMS = (Path(__file__).resolve().parents[4] / 'nav2' /
          'nav2_params_20261120.yaml')


@pytest.fixture(scope='module')
def costmap(tmp_path_factory):
    config = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    local = config['local_costmap']['local_costmap']
    config_path = tmp_path_factory.mktemp('wp3_costmap') / 'params.yaml'
    config_path.write_text(yaml.safe_dump({'costmap': {'costmap': local}}),
                           encoding='utf-8')
    log_path = config_path.with_suffix('.log')
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            ['ros2', 'run', 'nav2_costmap_2d', 'nav2_costmap_2d',
             '--ros-args', '--params-file', str(config_path)],
            stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        rclpy.init()
        node = Node('wp3_costmap_test')
        static_tf = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        static_tf.sendTransform(transform)

        node.grid = None
        node.create_subscription(OccupancyGrid, '/costmap/costmap',
                                 lambda msg: setattr(node, 'grid', msg), 10)
        node.scan = node.create_publisher(LaserScan, '/scan',
                                          qos_profile_sensor_data)
        node.range = node.create_publisher(Range, '/ultrasonic/front',
                                           qos_profile_sensor_data)
        node.cloud = node.create_publisher(PointCloud2,
                                           '/vision/ground_obstacles',
                                           qos_profile_sensor_data)
        client = node.create_client(ChangeState, '/costmap/costmap/change_state')
        try:
            assert client.wait_for_service(timeout_sec=15.0), log_path.read_text()
            for transition in (Transition.TRANSITION_CONFIGURE,
                               Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
                assert future.done() and future.result().success, log_path.read_text()
            yield node
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.terminate()
            process.wait(timeout=10.0)


def wait_for_cost(node, publish, x, y, *, marked):
    deadline = time.monotonic() + 8.0
    observed = None
    while time.monotonic() < deadline:
        publish()
        rclpy.spin_once(node, timeout_sec=0.1)
        grid = node.grid
        if grid is None:
            continue
        col = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
        row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
        if 1 <= col < grid.info.width - 1 and 1 <= row < grid.info.height - 1:
            observed = max(grid.data[r * grid.info.width + c]
                           for r in range(row - 1, row + 2)
                           for c in range(col - 1, col + 2))
            if (observed >= 90) == marked:
                return
    pytest.fail(f'Unexpected cost near ({x}, {y}); last cost={observed}')


def test_lidar_marks_the_local_costmap(costmap):
    def publish():
        scan = LaserScan()
        scan.header.stamp = costmap.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = math.pi / 2.0 - 0.01
        scan.angle_max = math.pi / 2.0 + 0.01
        scan.angle_increment = 0.01
        scan.range_min = 0.02
        scan.range_max = 12.0
        scan.ranges = [1.0, 1.0, 1.0]
        costmap.scan.publish(scan)

    wait_for_cost(costmap, publish, 0.0, 1.0, marked=True)


def test_ultrasonic_marks_the_local_costmap(costmap):
    def publish():
        reading = Range()
        reading.header.stamp = costmap.get_clock().now().to_msg()
        reading.header.frame_id = 'base_link'
        reading.radiation_type = Range.ULTRASOUND
        reading.field_of_view = 0.2618
        reading.min_range = 0.02
        reading.max_range = 4.5
        reading.range = 1.0
        costmap.range.publish(reading)

    wait_for_cost(costmap, publish, 1.0, 0.0, marked=True)


def test_projected_vision_marks_the_local_costmap(costmap):
    def publish():
        header = Header()
        header.stamp = costmap.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        costmap.cloud.publish(create_cloud_xyz32(header, [[1.0, 0.7, 0.1]]))

    wait_for_cost(costmap, publish, 1.0, 0.7, marked=True)


def publish_vision(costmap, points):
    header = Header()
    header.stamp = costmap.get_clock().now().to_msg()
    header.frame_id = 'base_link'
    costmap.cloud.publish(create_cloud_xyz32(header, points))


def test_vision_obstacle_disappears_when_the_next_cloud_is_empty(costmap):
    wait_for_cost(costmap, lambda: publish_vision(costmap, [[1.0, 0.7, 0.1]]),
                  1.0, 0.7, marked=True)

    def publish_empty():
        publish_vision(costmap, [])

    wait_for_cost(costmap, publish_empty, 1.0, 0.7, marked=False)


def test_clearing_vision_preserves_lidar_and_ultrasonic_obstacles(costmap):
    test_lidar_marks_the_local_costmap(costmap)
    test_ultrasonic_marks_the_local_costmap(costmap)
    wait_for_cost(costmap, lambda: publish_vision(costmap, [[1.0, 0.7, 0.1]]),
                  1.0, 0.7, marked=True)

    def publish_empty():
        publish_vision(costmap, [])

    wait_for_cost(costmap, publish_empty, 1.0, 0.7, marked=False)
    wait_for_cost(costmap, publish_empty, 0.0, 1.0, marked=True)
    wait_for_cost(costmap, publish_empty, 1.0, 0.0, marked=True)


def test_projected_vision_respects_the_minimum_range(costmap):
    deadline = time.monotonic() + 2.0
    received_grid = False
    while time.monotonic() < deadline:
        publish_vision(costmap, [[0.35, 0.3, 0.1]])
        rclpy.spin_once(costmap, timeout_sec=0.1)
        grid = costmap.grid
        if grid is None:
            continue
        col = math.floor((0.35 - grid.info.origin.position.x) / grid.info.resolution)
        row = math.floor((0.3 - grid.info.origin.position.y) / grid.info.resolution)
        observed = max(grid.data[r * grid.info.width + c]
                       for r in range(row - 1, row + 2)
                       for c in range(col - 1, col + 2))
        assert observed < 90
        received_grid = True
    assert received_grid and costmap.cloud.get_subscription_count() > 0


def test_a_timed_out_vision_source_marks_the_costmap_not_current(tmp_path):
    params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))
    values = params['local_costmap']['local_costmap']['ros__parameters']
    config_path = tmp_path / 'params.yaml'
    config_path.write_text(yaml.safe_dump({'costmap': {'costmap':
                           {'ros__parameters': values}}}), encoding='utf-8')
    result = subprocess.run(['vision_layer_probe', str(config_path)],
                            capture_output=True, text=True, timeout=15.0)
    assert result.returncode == 0, result.stdout + result.stderr
