"""lidar-ultra 改造区验收测试：运动方向门同源、超声 fail-closed、慢行区限速。

覆盖任务书 lidar-ultra 的验收点：
- /drive_safety 的 blocked 原因与自动命令（/auto_cmd_vel）同源、同拍；
- 雷达陈旧（原始 header.stamp 帧龄）→ 方向门 stale；
- 运动方向扇区距离 < 安全距离 → <dir>_blocked，且自动命令为零；
- 逆向/侧向命令按 rear/left/right 扇区判；
- 超声 0xFFFF→max_range = 无效 = 停车（ULTRA-3）；
- 慢行区（stop_distance < front < slow_distance）线速度 = 比例限速 × 比例
  （ULTRA-4）。

节点级测试用假 rclpy（test/fake_ros.py），纯逻辑部分只 import 纯模块。
"""

import json
import math

import pytest

import fake_ros  # 由 conftest 装载到 sys.modules
from opi_robot.fusion_node import safety_state as SS
from opi_robot.fusion_node.fusion_node import (
    FusionNode, frame_age_from_stamp, motion_direction_for)
from opi_robot.lidar_node import motion_check as MC
from test_fusion_node import all_around, diag, ring

GEOM = MC.make_geometry()        # 0.25 × 0.15，半角 30°


# ---------------------------------------------------------------- 纯逻辑

def test_direction_from_command():
    assert motion_direction_for(0.3, 0.0) == 'front'
    assert motion_direction_for(-0.3, 0.0) == 'rear'
    # Positive angular velocity is counter-clockwise in the current firmware.
    # 当前固件以逆时针角速度为正。
    assert motion_direction_for(0.0, 0.4) == 'left'
    assert motion_direction_for(0.0, -0.4) == 'right'
    assert motion_direction_for(0.0, 0.0) == 'front'     # 静止：默认前方
    assert motion_direction_for(-0.3, 0.4) == 'rear'     # 有前进/后退时以前后为准


def test_frame_age_from_stamp_unusable_is_none():
    class T:
        def __init__(self, s, n):
            self.sec, self.nanosec = s, n
    assert frame_age_from_stamp(None, 100.0) is None
    assert frame_age_from_stamp(T(0, 0), 100.0) is None        # 未填
    assert frame_age_from_stamp(T(10, 5 * 10 ** 8), 100.0) is None  # 时钟域不符
    age = frame_age_from_stamp(T(99, 9 * 10 ** 8), 100.0)
    assert age == pytest.approx(0.1)


# ---------------------------------------------------------------- 节点夹具

def _stamp(msg, node):
    now = node.get_clock().now_msg()
    msg.header.stamp = fake_ros.Time(sec=now.sec, nanosec=now.nanosec)
    return msg


def make_node(cruise=True, body=True):
    node = FusionNode()
    node.enable_cmd_vel = True
    if body:
        node.body_geometry['half_length'] = 0.25
        node.body_geometry['half_width'] = 0.15
    if cruise:
        node.on_cruise(fake_ros.Bool(data=True))
    return node


def feed(node, front=3.0, beam=None, us=0.8):
    """诊断 + 带时间戳扫描 + 超声（0.8 m）+ 证据增长两次。"""
    node.get_clock().advance(0.1)
    node.on_diagnostics(diag(fake_ros.DiagnosticStatus.OK))
    data = all_around(front)
    if beam is not None:
        deg, dist = beam
        data[str(deg)] = dist
    node.on_scan(_stamp(ring(**data), node))
    if us is not None:
        node.on_ultrasonic(fake_ros.Range(range=us, min_range=0.02,
                                          max_range=4.5))
    for count in (100, 101):
        node.on_evidence(fake_ros.String(data=json.dumps(
            {'v': 1, 'epoch': 7, 'frames_ok': count,
             'max_point_age_ms': None})))


def safety(node):
    msgs = node.publishers['drive_safety'].messages
    assert msgs
    return json.loads(msgs[-1].data)


def report(node):
    return json.loads(node.publishers['obstacle_report'].messages[-1].data)


def twist(node):
    msgs = node.publishers['auto_cmd_vel'].messages
    assert msgs
    return msgs[-1]


@pytest.mark.parametrize('beam,blocked_direction', [(180, 'rear'), (90, 'left'), (270, 'right')])
def test_ir_target_checks_reverse_and_turn_while_cruise_disabled(beam, blocked_direction):
    node = make_node(cruise=False)
    feed(node, beam=(beam, 0.3))
    if beam != 180:
        node.latest_scan.ranges[180] = 3.0
    wz = -0.2 if blocked_direction == 'right' else 0.2
    node.subscriptions['/dock/target'].callback(fake_ros.String(data=json.dumps(
        {'active': True, 'generation': 17, 'session': 123, 'vx': -0.1, 'wz': wz})))
    node.decide()
    result = safety(node)
    assert result['allowed'] is False
    assert result['dock']['generation'] == 17
    assert result['dock']['checks'][blocked_direction]['blocked'] is True


def test_ir_target_clear_reports_same_generation():
    node = make_node(cruise=False)
    feed(node)
    node.latest_scan.ranges[180] = 3.0
    node.subscriptions['/dock/target'].callback(fake_ros.String(data=json.dumps(
        {'active': True, 'generation': 18, 'session': 123, 'vx': -0.1, 'wz': 0.2})))
    node.decide()
    result = safety(node)
    assert result['allowed'] is True
    assert result['dock']['generation'] == 18
    assert result['dock']['session'] == 123
    assert set(result['dock']['checks']) == {'rear', 'left'}


def test_ir_target_expiry_and_front_ultrasonic_remain_blocking():
    node = make_node(cruise=False)
    feed(node, us=0.2)
    node.latest_scan.ranges[180] = 3.0
    node.subscriptions['/dock/target'].callback(fake_ros.String(data=json.dumps(
        {'active': True, 'generation': 19, 'session': 123, 'vx': -0.1, 'wz': 0.2})))
    node.decide()
    assert safety(node)['allowed'] is False
    assert safety(node)['ultrasonic']['deny'] is True

    node.get_clock().advance(0.31)
    node.decide()
    assert safety(node)['reason'] == 'dock_target_stale'


# ---------------------------------------------------------------- 同源

def test_clear_path_allows_and_drives():
    node = make_node()
    feed(node)
    node.decide()
    st = safety(node)
    assert st['allowed'] is True and st['reason'] == 'ok'
    assert st['motion']['blocked'] is False
    assert st['motion']['direction'] == 'front'
    assert st['motion']['ranges']['front'] == pytest.approx(3.0)
    assert twist(node).linear.x == pytest.approx(0.4)


def test_same_motion_object_in_safety_and_report():
    """页面/桥/nav relay 读的是同一份判定：/drive_safety 与 obstacle_report。"""
    node = make_node()
    feed(node, front=0.3)
    node.decide()
    assert safety(node)['motion'] == report(node)['motion']


def test_calibrated_longitudinal_offset_blocks_near_front_obstacle(monkeypatch):
    overrides = {'body_half_length': 0.20, 'body_half_width': 0.20,
                 'body_center_x': -0.08}
    declare = FusionNode.declare_parameter

    def declare_with_overrides(node, name, value=None):
        return declare(node, name, overrides.get(name, value))

    monkeypatch.setattr(FusionNode, 'declare_parameter', declare_with_overrides)
    node = make_node(body=False)
    feed(node, beam=(0, 0.15))
    node.decide()

    assert safety(node)['motion']['reason'] == 'front_blocked'
    assert twist(node).linear.x == 0.0
    assert report(node)['body']['center_x'] == pytest.approx(-0.08)


def test_front_blocked_denies_with_motion_reason_and_zero_cmd():
    node = make_node()
    # 45° 车体角恰在 30° 半角扇区的空隙里：两侧 3.0 m 覆盖下仍须拦停
    feed(node, front=3.0, beam=(45, 0.3))
    node.decide()
    st = safety(node)
    assert st['allowed'] is False
    assert st['reason'] == 'motion_front_gap_blocked'
    assert st['motion']['blocked'] is True
    assert st['motion']['ranges']['front'] == pytest.approx(3.0)
    assert twist(node).linear.x == 0.0

    # 0.3 m 小于动作层 stop_distance：许可由动作层先拒（具体原因
    # obstacle_stop 优先于笼统的 motion_front_blocked），速度为零。
    node = make_node()
    feed(node, front=0.3)
    node.decide()
    assert safety(node)['reason'] == 'obstacle_stop'
    assert twist(node).linear.x == 0.0


def test_cruise_side_obstacle_blocks_forward():
    """直线巡航、侧向极近障碍（repro1：左 +90° 0.3 m、前 3 m）必须拦停。"""
    node = make_node()
    feed(node, front=3.0, beam=(90, 0.3))
    node.decide()
    st = safety(node)
    assert st['motion']['direction'] == 'front'
    assert st['motion']['blocked'] is True
    assert st['motion']['reason'] == 'side_blocked'
    assert st['motion']['ranges']['side'] == pytest.approx(0.3)
    assert twist(node).linear.x == 0.0


def test_turn_side_obstacle_blocks():
    """无巡航意图、转角命令：方向门按该侧扇区判（未在标定帧内判两侧）。"""
    for command, direction in (((0.0, 0.4), 'left'), ((0.0, -0.4), 'right')):
        node = make_node(cruise=False)
        feed(node, front=3.0)
        node._last_cmd = command
        verdict = node.motion_verdict(active=False)
        assert verdict['direction'] == direction, command
        result = node.evaluate(node.now_s())
        result['linear'], result['angular'] = command
        result.update(node._command(result['motion'], *command))
        assert result['motion']['direction'] == direction, command
        assert result['motion']['ranges']['side'] is not None


def test_stale_header_stamp_blocks():
    node = make_node()
    feed(node)
    node.latest_scan.header.stamp = fake_ros.Time(sec=0, nanosec=0)
    node.decide()
    st = safety(node)
    assert st['motion']['blocked'] is True
    assert st['motion']['reason'] == 'stale'
    assert st['reason'] == 'motion_stale'
    assert twist(node).linear.x == 0.0


def test_uncalibrated_body_blocks():
    node = make_node(body=False)
    feed(node)
    node.decide()
    st = safety(node)
    assert st['motion']['reason'] == 'front_no_geometry'
    assert report(node)['body']['missing'] == ['half_length', 'half_width']


@pytest.mark.parametrize('command,direction,beam_deg', [
    ((-0.3, 0.0), 'rear', 180),
    ((0.0, 0.4), 'left', 90),
    ((0.0, -0.4), 'right', 270),
])
def test_reverse_and_turn_use_their_own_sector(command, direction, beam_deg):
    node = make_node()
    feed(node, beam=(beam_deg, 0.3))           # 该方向扇区里的近点
    node.decide()
    result = node.evaluate(node.now_s())
    result['linear'], result['angular'] = command
    result.update(node._command(result['motion'], *command))
    assert result['motion']['direction'] == direction
    assert result['motion']['blocked'] is True
    assert result['motion']['reason'] == '%s_blocked' % direction


def test_cruise_off_is_not_blocked():
    """巡航关：无自动运动意图，方向门标 inactive，不拦停。"""
    node = make_node(cruise=False)
    feed(node, front=0.0 + 3.0)
    node.decide()
    st = safety(node)
    assert st['motion']['active'] is False
    assert st['motion']['reason'] == 'inactive'
    assert twist(node).linear.x == 0.0


# ---------------------------------------------------------------- 超声 fail-closed

def test_ultrasonic_no_echo_far_is_ok():
    """[2026-11-18 用户定案] 0xFFFF → max_range（4.5）= 波束打空 = 远处
    无约束：不锁车、许可放行；近距停车仍由迟滞门管。"""
    node = make_node()
    feed(node, us=4.5)
    node.decide()
    st = safety(node)
    assert st['allowed'] is True
    assert st['ultrasonic']['valid'] is True
    assert st['ultrasonic']['deny'] is False


def test_ultrasonic_never_heard_denies():
    node = make_node()
    feed(node, us=None)
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_invalid'


def test_ultrasonic_short_gap_does_not_latch_stale():
    node = make_node()
    feed(node, us=0.8)
    node.decide()
    assert safety(node)['allowed'] is True
    node.get_clock().advance(0.45)
    feed(node, us=None)  # 最后一次超声读数距现在 0.55 秒
    node.decide()
    us = safety(node)['ultrasonic']
    assert us['valid'] is True and us['fault'] is False
    assert us['timeout_s'] == pytest.approx(0.6)
    assert safety(node)['allowed'] is True
    feed(node, us=0.8)
    node.decide()
    assert safety(node)['allowed'] is True


def test_ultrasonic_long_gap_latches_until_rearmed():
    node = make_node()
    feed(node, us=0.8)
    node.get_clock().advance(0.55)
    feed(node, us=None)  # 最后一次超声读数距现在 0.65 秒
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_invalid'
    assert safety(node)['ultrasonic']['fault_reason'] == 'stale'
    feed(node, us=0.8)
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_invalid'
    node.on_ultrasonic_rearm(fake_ros.Bool(data=True))
    feed(node, us=0.8)
    node.decide()
    assert safety(node)['allowed'] is True


def test_ultrasonic_near_stop_remains_during_tolerated_gap():
    node = make_node()
    feed(node, us=0.3)
    node.get_clock().advance(0.45)
    feed(node, us=None)
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_stop'
    assert safety(node)['ultrasonic']['valid'] is True


def test_ultrasonic_near_reading_stops_after_tolerated_gap():
    node = make_node()
    feed(node, us=0.8)
    node.get_clock().advance(0.45)
    feed(node, us=None)
    node.decide()
    assert safety(node)['allowed'] is True
    node.on_ultrasonic(fake_ros.Range(range=0.3, min_range=0.02,
                                      max_range=4.5))
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_stop'


def test_ultrasonic_bad_reading_stops_during_tolerated_gap():
    node = make_node()
    feed(node, us=0.8)
    node.get_clock().advance(0.45)
    feed(node, us=float('nan'))
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_invalid'
    assert safety(node)['ultrasonic']['fault_reason'] == 'no_echo'


def test_ultrasonic_recovery_requires_restart():
    """坏数据(NaN)无效即停：恢复后仍需"重新启动"；有效读数不能自行解除。"""
    gate = SS.UltrasonicGate(0.4, 0.5, 0.5, invalid_at=4.5)
    gate.feed(float('nan'), 1.0)                # 坏数据
    assert gate.verdict(1.1)[2] is False        # us_valid False
    gate.feed(0.9, 1.2)                         # 读数自己恢复也不算
    stop, _rng, valid, _age = gate.verdict(1.3)
    assert valid is False and stop is True
    gate.rearm(1.4)                             # "重新启动"
    gate.feed(0.9, 1.5)
    stop, _rng, valid, _age = gate.verdict(1.6)
    assert valid is True and stop is False


def test_node_ultrasonic_rearm_topic():
    """fusion_node 订阅 ultrasonic_rearm：只认 Bool(True)，False 忽略。"""
    node = make_node()
    feed(node, us=float('nan'))                 # [2026-11-18] 坏数据 → 无效
    node.decide()
    assert safety(node)['reason'] == 'ultrasonic_invalid'
    assert 'ultrasonic_rearm' in node.subscriptions
    node.on_ultrasonic_rearm(fake_ros.Bool(data=False))
    assert node.us_gate.rearms == 0 and node.us_gate.fault is True
    node.on_ultrasonic_rearm(fake_ros.Bool(data=True))
    assert node.us_gate.rearms == 1 and node.us_gate.fault is False
    # 复位后新读数（> release）即可放行
    feed(node, us=0.8)
    node.decide()
    st = safety(node)
    assert st['ultrasonic']['valid'] is True and st['reason'] == 'ok'


def test_web_us_view_never_says_pass_when_invalid():
    """网页状态负载只转述服务端：无效时 ultrasonic.valid=False / deny=True。"""
    node = make_node()
    feed(node, us=float('nan'))                # [2026-11-18] 坏数据 → 无效
    node.decide()
    us = report(node)['ultrasonic']
    assert us['valid'] is False and us['deny'] is True
    html = open('opi_robot/web_node/console_page.html', encoding='utf-8').read()
    assert "us.fault === true || us.valid === false" in html
    assert "usM !== null ? '通行'" not in html


# ---------------------------------------------------------------- 慢行区

def test_slow_zone_applies_computed_limit():
    """0.6 m 在 (0.4, 0.9) 慢行区：ratio=0.4、×slow_min_ratio 0.5 ⇒ 0.08 m/s。"""
    node = make_node()
    feed(node, front=0.6)
    node.decide()
    rep = report(node)
    assert rep['action'] == 'slow'
    assert rep['linear'] == pytest.approx(0.4 * 0.4 * 0.5)
    assert twist(node).linear.x == pytest.approx(0.08)
    assert safety(node)['allowed'] is True     # 慢行不是停车


def test_clear_near_slow_distance_is_limited_too():
    node = make_node()
    feed(node, front=1.0)
    node.decide()
    # 前方 1.0 m ≥ slow_distance 0.9：classify=clear；但方向门读同一份
    # 判定，1.0 ≥ 0.9 不减速
    assert twist(node).linear.x == pytest.approx(0.4)


def test_far_clear_full_speed():
    node = make_node()
    feed(node, front=3.0)
    node.decide()
    assert twist(node).linear.x == pytest.approx(0.4)


def test_b7_motion_direction_matches_guard_command_layer():
    """Use the firmware's counter-clockwise-positive convention.
    使用固件的逆时针为正约定。
    """
    assert motion_direction_for(0.0, 0.4) == 'left'
    assert motion_direction_for(0.0, -0.4) == 'right'
    assert motion_direction_for(0.3, 0.4) == 'front'   # 前进为主仍以前为准
    assert motion_direction_for(-0.3, 0.4) == 'rear'


def test_b7_turn_sector_agrees_with_guard_side():
    """A left-side obstacle blocks a positive angular command.
    左侧障碍物阻止正角速度命令。
    """
    node = make_node()
    feed(node, front=3.0, beam=(90, 0.3))
    node.decide()
    result = node.evaluate(node.now_s())
    result.update(node._command(result['motion'], 0.0, 0.4))
    verdict = result['motion']
    assert verdict['direction'] == 'left'
    assert verdict['blocked'] is True, verdict['reason']
