"""WP3 charging and patrol integration tests without a ROS installation.
无 ROS 安装时验证 WP3 回充和巡逻接线。
"""
import pytest

from opi_robot.web_node import patrol as P
from opi_robot.web_node import nav_goal as NG
from opi_robot.web_node import web_debug as W


def test_patrol_maps_nav_sequence_after_an_unrelated_goal(monkeypatch, tmp_path):
    ctrl = NG.NavGoalControl()
    assert ctrl.attach(None)
    monkeypatch.setattr(NG, '_NAV', ctrl)
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.05\n', encoding='utf8')
    (tmp_path / 'map.pgm').write_bytes(b'map')
    W.configure_patrol(reset_runner=True, map_yaml=str(path))
    mid = W.patrol_map_id(str(path))
    route, err = P.validate_route([[0, 0], [1, 0]], mid, mid)
    assert not err
    W._patrol['runner'].save(route)
    W._patrol['license'].accept(1, 0)
    assert W._patrol['runner'].rearm(license_ok=True)[0]
    assert ctrl.request_goal(2, 0, 0)[0]
    ctrl._client.handles[-1].finish(4)
    assert W.patrol_start()[0]
    W._patrol_run_dispatch()
    runner = W._patrol['runner']
    assert runner.pending is not None
    W.patrol_stop()
    assert ctrl._client.handles[-1].cancelled
    ctrl._client.handles[-1].finish(5)
    assert runner.cancel_request is None


def test_pause_waits_for_cancel_and_resume_keeps_waypoint():
    seq = iter([1, 2, 3])
    goals = []
    def dispatch(goal):
        goals.append(goal)
        return next(seq)
    runner = P.PatrolRunner(dispatch, lambda *_: True, lambda *_: None, lambda: 0)
    route, err = P.validate_route([[0, 0], [1, 0]], 'map', 'map')
    assert not err
    runner.save(route)
    runner.rearm(license_ok=True)
    runner.start('map')
    runner.on_accepted(1)
    runner.on_result(1, 'succeeded')
    runner.on_accepted(2)
    assert runner.index == 1
    assert runner.pause()[0]
    assert runner.cancel_request == 2
    assert not runner.resume('map', license_ok=True)[0]
    runner.on_result(2, 'cancelled')
    assert runner.cancel_request is None
    assert runner.state == 'paused'
    assert runner.resume('map', license_ok=True)[0]
    assert runner.index == 1 and goals[-1]['x'] == 1


class Clock:
    value = 0.0
    def __call__(self):
        return self.value
    def advance(self, seconds):
        self.value += seconds


class License:
    def fresh(self):
        return True


class Nav:
    def __init__(self):
        self.sent = []
    def view(self):
        return {'state': 'idle'}
    def request_goal(self, x, y, yaw, on_reserved=None):
        seq = 42
        on_reserved(seq)
        self.sent.append((x, y, yaw))
        return True, {'seq': seq}
    def cancel_seq(self, seq, result_known=True):
        return True


def config():
    return dict(prep_x=1.0, prep_y=2.0, prep_yaw=0.0, low_voltage=22.0,
                current_min=100.0, current_tolerance=50.0, stable_window=0.4,
                undock_distance=0.3, undock_speed=0.1, undock_timeout=5.0,
                confirm_timeout=2.0)


def test_charging_waits_for_cancel_telemetry_full_and_fresh_odometry():
    from opi_robot.web_node.charging import ChargingCoordinator
    clock = Clock()
    nav = Nav()
    runner = P.PatrolRunner(lambda _: 1, lambda *_: True, lambda *_: None, clock)
    runner.save({'map_id': 'map', 'points': [[0, 0, 0], [1, 0, 0]]})
    runner.rearm(license_ok=True)
    runner.start('map')
    runner.on_accepted(1)
    services, holds, speeds = [], [], []
    c = ChargingCoordinator(config(), runner, License(), nav, lambda: 'map',
                            services.append, holds.append, speeds.append,
                            lambda: True, clock)
    c.on_voltage(21.0)
    c.tick()
    assert c.state == 'pausing' and nav.sent == []
    runner.on_result(1, 'cancelled')
    c.tick()
    assert c.state == 'navigating' and len(nav.sent) == 1
    c.on_nav_result(42, 'succeeded', '')
    c.tick()
    assert services == [True]
    c.on_dock_result(True)
    c.on_dock(True, True, 120, True)
    c.tick()
    assert c.state == 'docking'
    # Ignore a full signal received before stable charging.
    # 充电稳定前收到的充满信号不计入本次充电。
    c.on_full(True)
    for _ in range(3):
        clock.advance(0.2)
        c.on_dock(True, True, 120, True)
        c.tick()
    assert c.state == 'charging'
    c.tick()
    assert services == [True]
    c.on_full(True)
    c.tick()
    assert services == [True, False]
    c.on_dock_result(True)
    c.on_dock(False, False, 0, True)
    c.on_odom(0.0, 0.0, 0.0)
    c.tick()
    assert c.state == 'undocking'
    clock.advance(0.1)
    c.on_dock(False, False, 0, True)
    c.on_odom(0.31, 0.0, 0.0)
    c.tick()
    assert c.state == 'idle' and runner.index == 0
    assert speeds[-1] == 0.0


def test_charging_requires_all_parameters():
    from opi_robot.web_node.charging import validate_config
    values = config()
    del values['undock_distance']
    with pytest.raises(ValueError, match='charging_undock_distance_m'):
        validate_config(values)


def test_manual_abort_during_enable_write_requests_disable_after_reply():
    from opi_robot.web_node.charging import ChargingCoordinator
    clock = Clock()
    runner = P.PatrolRunner(lambda _: 1, lambda *_: True, lambda *_: None, clock)
    runner.rearm(license_ok=True)
    commands = []
    c = ChargingCoordinator(config(), runner, License(), Nav(), lambda: 'map',
                            commands.append, lambda _: None, lambda _: None,
                            lambda: True, clock)
    c.state = 'prep_arrived'
    c.tick()
    assert commands == [True]
    c.abort('manual_takeover')
    assert c.state == 'failed' and runner.locked
    c.on_dock_result(True)
    assert commands == [True, False]


def test_undock_tick_publishes_only_one_forward_command():
    from opi_robot.web_node.charging import ChargingCoordinator
    clock = Clock()
    runner = P.PatrolRunner(lambda _: 1, lambda *_: True, lambda *_: None, clock)
    runner.rearm(license_ok=True)
    speeds = []
    c = ChargingCoordinator(config(), runner, License(), Nav(), lambda: 'map',
                            lambda _: None, lambda _: None, speeds.append,
                            lambda: True, clock)
    c.holding = True
    c.state = 'undocking'
    c.undock_started = 0.0
    c.origin = c.previous = (0.0, 0.0, 0.0)
    c.on_dock(False, False, 0, True)
    c.on_odom(0.01, 0, 0)
    c.on_full(True)
    c.tick()
    assert speeds == [0.1]


def test_pause_before_dispatch_never_sends_cancelled_goal(monkeypatch):
    ctrl = NG.NavGoalControl()
    assert ctrl.attach(None)
    monkeypatch.setattr(NG, '_NAV', ctrl)
    W.configure_patrol(reset_runner=True)
    runner = W._patrol['runner']
    runner.save({'map_id': 'map', 'points': [[0, 0, 0], [1, 0, 0]]})
    runner.rearm(license_ok=True)
    assert runner.start('map')[0]
    assert runner.pause()[0]
    W._patrol_run_dispatch()
    assert not ctrl._client.goals
    assert runner.cancel_request is None


def test_pause_timeout_keeps_hold_and_locks():
    from opi_robot.web_node.charging import ChargingCoordinator
    clock = Clock()
    nav = Nav()
    runner = P.PatrolRunner(lambda _: 1, lambda *_: True, lambda *_: None, clock)
    runner.save({'map_id': 'map', 'points': [[0, 0, 0], [1, 0, 0]]})
    runner.rearm(license_ok=True)
    runner.start('map')
    runner.on_accepted(1)
    holds, speeds = [], []
    c = ChargingCoordinator(config(), runner, License(), nav, lambda: 'map',
                            lambda _: None, holds.append, speeds.append,
                            lambda: True, clock)
    c.on_voltage(21.0)
    c.tick()
    clock.advance(2.1)
    c.tick()
    assert c.state == 'failed' and runner.locked
    assert not nav.sent and holds[-1] is True and speeds[-1] == 0.0


def test_enabled_ros_adapter_receives_and_publishes_expected_messages(monkeypatch):
    import sys
    import types
    import fake_ros
    from opi_robot.web_node import charging
    services = types.ModuleType('std_srvs.srv')
    class SetBool:
        Request = type('Request', (), {'data': False})
    class Trigger:
        Request = type('Request', (), {})
    services.SetBool, services.Trigger = SetBool, Trigger
    monkeypatch.setitem(sys.modules, 'std_srvs', types.ModuleType('std_srvs'))
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', services)
    monkeypatch.setattr(sys.modules['nav_msgs.msg'], 'Odometry', type('Odometry', (), {}), raising=False)

    class Node(fake_ros.FakeNode):
        def create_client(self, *_):
            return types.SimpleNamespace(service_is_ready=lambda: True,
                                         call_async=lambda _: fake_ros._ActionFuture(
                                             result=types.SimpleNamespace(success=True)))
        def create_service(self, _type, topic, callback):
            self.service = (topic, callback)
            return self.service

    node = Node('web_bridge')
    runner = P.PatrolRunner(lambda _: 1, lambda *_: True, lambda *_: None, lambda: 0)
    runner.rearm(license_ok=True)
    ros = charging.ChargingRos(node, runner, License(), Nav(), lambda: 'map',
                               lambda: True, config())
    assert {'/diagnostics', '/chassis/dock_status', '/battery/full', '/odom'} <= set(node.subscriptions)
    assert node.service[0] == '/charging/request'
    status = fake_ros.DiagnosticStatus.make(values=[
        fake_ros.KeyValue(key='charging', value='true'),
        fake_ros.KeyValue(key='allow_recharge', value='true'),
        fake_ros.KeyValue(key='dock_hw_present', value='true'),
        fake_ros.KeyValue(key='charging_current_ma', value='120')])
    node.subscriptions['/chassis/dock_status'].callback(status)
    assert ros.coordinator.dock == (True, True, 120, True)
    ros._publish_hold(True)
    ros._publish_speed(0.1)
    assert node.publishers['/charging/nav_hold'].messages[-1].data is True
    assert node.publishers['/charging/cmd_vel'].messages[-1].linear.x == 0.1


def test_disabled_charging_attaches_no_ros_interfaces():
    import fake_ros
    from opi_robot.web_node.charging import attach
    node = fake_ros.FakeNode('web_bridge')
    assert attach(node, None, None, None, None, None) is None
    assert not node.publishers and not node.subscriptions and not node.timers
