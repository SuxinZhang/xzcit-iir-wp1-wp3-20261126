"""Regression tests for the patrol-to-inspection handoff.
巡逻到巡检抓拍交接的回归测试。

These tests exercise the pure patrol state machine hook only; they do not
require ROS 2, a camera, or a running RKNN node.
这些测试仅覆盖纯巡逻状态机钩子，不依赖 ROS 2、相机或运行中的 RKNN 节点。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[6]
OPI_ROOT = ROOT / 'Master/ROS/ros2_ws/src/opi_robot'
if str(OPI_ROOT) not in sys.path:
    sys.path.insert(0, str(OPI_ROOT))

from opi_robot.web_node import patrol as P  # noqa: E402


class Clock:
    """Deterministic monotonic clock for the patrol state machine.
    巡逻状态机使用的确定性单调时钟。
    """

    def __init__(self):
        self.now = 1.0

    def __call__(self):
        return self.now


class FakeNav:
    """Minimal navigation adapter recording dispatched goals.
    记录已下发目标的最小导航适配器。
    """

    def __init__(self):
        self.next_id = 1
        self.goals = []

    def dispatch(self, goal):
        request_id = self.next_id
        self.next_id += 1
        self.goals.append(dict(goal))
        return request_id

    def cancel(self, _request_id, _result_known=True):
        return True

    def lock_stop(self, _reason):
        """Accept lock requests without side effects.
        接受锁停请求但不产生副作用。
        """
        return None


def _route():
    route, error = P.validate_route([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], 'map-a', 'map-a')
    assert error is None
    return route


def test_hook_fires_after_waypoint_success_and_patrol_continues():
    """Hook receives the arrived waypoint and patrol dispatches the next goal.
    钩子收到已到达航点，巡逻继续下发下一个目标。
    """
    nav = FakeNav()
    calls = []
    runner = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, Clock(),
                            on_waypoint=lambda index, point: calls.append((index, point)))
    runner.save(_route())
    assert runner.start('map-a')[0]
    first = runner.sending
    assert first is not None
    runner.on_accepted(first)
    runner.on_result(first, 'succeeded')
    assert calls == [(0, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})]
    assert len(nav.goals) == 2


def test_hook_failure_does_not_abort_patrol():
    """A throwing vision hook must be isolated from navigation state transitions.
    抛出异常的视觉钩子必须与导航状态迁移隔离。
    """
    nav = FakeNav()

    def broken(_index, _point):
        raise RuntimeError('vision unavailable')

    runner = P.PatrolRunner(nav.dispatch, nav.cancel, nav.lock_stop, Clock(),
                            on_waypoint=broken)
    runner.save(_route())
    assert runner.start('map-a')[0]
    first = runner.sending
    runner.on_accepted(first)
    runner.on_result(first, 'succeeded')
    assert len(nav.goals) == 2
    assert runner.state in ('running', 'holding_goal')
