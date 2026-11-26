"""Two-stage charging coordinator.
两段式回充协调器。
"""

import json
import math
import threading
import time
from collections import deque


FRESH_S = 0.5
PARAMETERS = {
    'prep_x': 'charging_prep_x_m',
    'prep_y': 'charging_prep_y_m',
    'prep_yaw': 'charging_prep_yaw_rad',
    'low_voltage': 'charging_low_voltage_v',
    'current_min': 'charging_min_current_ma',
    'current_tolerance': 'charging_current_tolerance_ma',
    'stable_window': 'charging_stable_window_s',
    'undock_distance': 'charging_undock_distance_m',
    'undock_speed': 'charging_undock_speed_m_s',
    'undock_timeout': 'charging_undock_timeout_s',
    'confirm_timeout': 'charging_confirmation_timeout_s',
}


def validate_config(config):
    """Reject missing or non-finite configuration before enabling motion.
    启用运动前拒绝缺失或非有限参数。
    """
    for key in PARAMETERS:
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('回充参数缺失或无效: ' + PARAMETERS[key])
    for key in ('low_voltage', 'current_min', 'current_tolerance', 'stable_window',
                'undock_distance', 'undock_speed', 'undock_timeout', 'confirm_timeout'):
        if config[key] <= 0:
            raise ValueError('回充参数必须大于零: ' + PARAMETERS[key])


class ChargingCoordinator:
    """Coordinate Nav2, firmware handoff and a guarded straight undock.
    协调 Nav2、固件移交和受安全门保护的直线退桩。
    """

    def __init__(self, config, runner, license_monitor, nav, map_id, send_dock,
                 publish_hold, publish_speed, guard_ok, now=time.monotonic,
                 lock=None):
        validate_config(config)
        self.config = dict(config)
        self.runner = runner
        self.license = license_monitor
        self.nav = nav
        self.map_id = map_id
        self.send_dock = send_dock
        self.publish_hold = publish_hold
        self.publish_speed = publish_speed
        self.guard_ok = guard_ok
        self.now = now
        self.lock = lock or threading.RLock()
        self.state = 'idle'
        self.reason = ''
        self.stage_started = None
        self.voltage = None
        self.voltage_t = None
        self.dock = None
        self.dock_t = None
        self.dock_seq = 0
        self.dock_after = 0
        self.full = False
        self.full_t = None
        self.full_seq = 0
        self.full_after = 0
        self.odom = None
        self.odom_t = None
        self.odom_seq = 0
        self.odom_used = 0
        self.distance = 0.0
        self.goal_seq = None
        self.goal_pending = False
        self.resume_patrol = False
        self.service_pending = False
        self.service_enable = None
        self.disable_after_write = False
        self.confirm_started = None
        self.current_samples = deque()
        self.requested = False
        self.holding = False
        self.reset_requested = False

    def _fresh(self, at):
        return at is not None and 0 <= self.now() - at <= FRESH_S

    def _safe(self):
        return self.license.fresh() and not self.runner.locked and self.guard_ok()

    def on_voltage(self, voltage):
        with self.lock:
            if voltage is None or not math.isfinite(voltage) or voltage <= 0:
                self.voltage = self.voltage_t = None
            else:
                self.voltage, self.voltage_t = voltage, self.now()

    def on_dock(self, charging, enabled, current, hardware):
        with self.lock:
            self.dock_seq += 1
            self.dock_t = self.now()
            self.dock = (charging, enabled, current, hardware)
            if self.state == 'docking':
                samples = self.current_samples
                if samples and self.dock_t - samples[-1][0] > FRESH_S:
                    samples.clear()
                if not charging or not enabled or not hardware or current < self.config['current_min']:
                    samples.clear()
                else:
                    samples.append((self.dock_t, current))
                    while samples and self.dock_t - samples[0][0] > self.config['stable_window'] + FRESH_S:
                        samples.popleft()
                    values = [v for _, v in samples]
                    if max(values) - min(values) > self.config['current_tolerance']:
                        samples.clear()
                        samples.append((self.dock_t, current))

    def on_full(self, value):
        with self.lock:
            self.full_seq += 1
            self.full = bool(value)
            self.full_t = self.now()

    def invalidate_dock(self):
        with self.lock:
            self.dock_t = None

    def on_odom(self, x, y, yaw):
        with self.lock:
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                self.odom_t = None
                return
            self.odom = (x, y, yaw)
            self.odom_t = self.now()
            self.odom_seq += 1

    def request(self):
        with self.lock:
            if self.state != 'idle' or not self._safe():
                return False, '回充不可启动：状态或运动许可无效'
            self.requested = True
            return True, '已请求回充'

    def _hold(self):
        self.holding = True
        self.publish_hold(True)
        self.publish_speed(0.0)

    def _service(self, enable):
        self.service_pending = True
        self.service_enable = enable
        self.confirm_started = self.now()
        self.send_dock(enable)

    def on_dock_result(self, success):
        with self.lock:
            if not self.service_pending:
                return
            self.service_pending = False
            if not success:
                if self.state != 'failed':
                    self._fail('红外回充服务写入失败')
                elif self.disable_after_write:
                    self.disable_after_write = False
                    self._service(False)
                return
            if self.disable_after_write:
                self.disable_after_write = False
                self._service(False)
                return
            self.dock_after = self.dock_seq
            self.confirm_started = self.now()
            if self.state == 'enable_write':
                self.state = 'enable_confirm'
            elif self.state == 'disable_write':
                self.state = 'disable_confirm'

    def on_nav_accepted(self, seq):
        with self.lock:
            if seq == self.goal_seq:
                self.goal_pending = True

    def on_nav_result(self, seq, status, reason=''):
        with self.lock:
            if seq != self.goal_seq:
                return
            self.goal_pending = False
            if self.state == 'navigating':
                if status == 'succeeded':
                    self.state = 'prep_arrived'
                else:
                    self._fail('预备点导航失败: ' + (reason or status))

    def _fail(self, reason):
        was_ir = self.state in ('enable_write', 'enable_confirm', 'docking',
                                'charging', 'disable_write', 'disable_confirm',
                                'undocking')
        self.reason = reason
        self.state = 'failed'
        self.requested = False
        self._hold()
        self.runner.lock_stop(reason)
        if self.goal_pending:
            self.nav.cancel_seq(self.goal_seq, result_known=False)
        if self.runner.state == 'paused':
            self.runner.state = 'stopped'
            self.runner.stop_reason = reason
        if was_ir:
            if self.service_pending and self.service_enable:
                self.disable_after_write = True
            elif not self.service_pending:
                self._service(False)

    def abort(self, reason):
        with self.lock:
            if self.state == 'idle':
                return
            self._fail(reason)

    def operator_rearm(self):
        with self.lock:
            if self.state == 'failed':
                self.reset_requested = True

    def _begin_nav(self):
        if self.nav.view()['state'] in ('sending', 'active'):
            self._fail('已有导航目标')
            return
        self.state = 'navigating'
        self.stage_started = self.now()
        self.goal_pending = True
        def reserved(seq):
            self.goal_seq = seq
        ok, view = self.nav.request_goal(self.config['prep_x'], self.config['prep_y'],
                                         self.config['prep_yaw'], on_reserved=reserved)
        if not ok:
            self.goal_pending = False
            self._fail('预备点导航发送失败: ' + str(view.get('error', '')))

    def tick(self):
        with self.lock:
            now = self.now()
            if self.holding:
                self.publish_hold(True)
                if self.state != 'undocking':
                    self.publish_speed(0.0)
            if self.state == 'failed':
                if (self.reset_requested and self._safe() and not self.goal_pending
                        and not self.service_pending and self._fresh(self.dock_t)
                        and self.dock is not None and not self.dock[1]):
                    self.publish_hold(False)
                    self.holding = False
                    self.reset_requested = False
                    self.state = 'idle'
                return
            if self.state != 'idle' and not self._safe():
                self.abort('网页许可或底盘安全门失效')
                return
            if self.state in ('disable_write', 'disable_confirm', 'undocking') and (
                    not self._fresh(self.full_t) or not self.full):
                self._fail('充满信号失效')
                return
            if self.state == 'idle':
                low = (self._fresh(self.voltage_t) and self.voltage is not None
                       and self.voltage <= self.config['low_voltage'])
                if not (self.requested or low) or not self._safe():
                    return
                self.requested = False
                self.resume_patrol = self.runner.state in ('running', 'holding_goal')
                if self.resume_patrol:
                    ok, message = self.runner.pause()
                    if not ok:
                        self._fail(message)
                        return
                    self.state = 'pausing'
                    self.stage_started = now
                else:
                    self._begin_nav()
            elif self.state == 'pausing':
                if self.runner.state != 'paused':
                    self._fail('巡逻暂停被中断')
                elif self.runner.cancel_request is None:
                    self._begin_nav()
                elif now - self.stage_started > self.config['confirm_timeout']:
                    self._fail('巡逻取消确认超时')
            elif self.state == 'navigating':
                if now - self.stage_started > self.config['confirm_timeout']:
                    self._fail('预备点导航超时')
            elif self.state == 'prep_arrived':
                self._hold()
                self.state = 'enable_write'
                self._service(True)
            elif self.state in ('enable_write', 'enable_confirm', 'disable_write', 'disable_confirm'):
                if now - self.confirm_started > self.config['confirm_timeout']:
                    self._fail('红外回充确认超时')
                elif self.state == 'enable_confirm' and self.dock_seq > self.dock_after and self._fresh(self.dock_t):
                    if self.dock[1] and self.dock[3]:
                        self.current_samples.clear()
                        self.state = 'docking'
                elif self.state == 'disable_confirm' and self.dock_seq > self.dock_after and self._fresh(self.dock_t):
                    if not self.dock[1] and self._fresh(self.odom_t):
                        self.origin = self.odom
                        self.previous = self.odom
                        self.odom_used = self.odom_seq
                        self.distance = 0.0
                        self.undock_started = now
                        self.state = 'undocking'
            elif self.state == 'docking':
                if not self._fresh(self.dock_t):
                    self._fail('回充遥测过期')
                elif (self.current_samples and
                      now - self.current_samples[0][0] >= self.config['stable_window']):
                    self.full_after = self.full_seq
                    self.state = 'charging'
            elif self.state == 'charging':
                if not self._fresh(self.dock_t) or not self.dock[0] or not self.dock[1]:
                    self._fail('充电遥测失效')
                elif self.full_seq > self.full_after and self.full and self._fresh(self.full_t):
                    self.state = 'disable_write'
                    self._service(False)
            elif self.state == 'undocking':
                if not self._fresh(self.odom_t) or not self._fresh(self.dock_t) or self.dock[1]:
                    self._fail('退桩里程计或红外遥测失效')
                elif now - self.undock_started > self.config['undock_timeout']:
                    self._fail('退桩超时')
                else:
                    if self.odom_seq != self.odom_used:
                        x, y, _ = self.odom
                        px, py, _ = self.previous
                        yaw = self.origin[2]
                        self.distance += (x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)
                        self.previous = self.odom
                        self.odom_used = self.odom_seq
                    if self.distance >= self.config['undock_distance']:
                        self.publish_speed(0.0)
                        if self.resume_patrol:
                            ok, message = self.runner.resume(self.map_id(), license_ok=self.license.fresh())
                            if not ok:
                                self._fail(message)
                                return
                        self.publish_hold(False)
                        self.holding = False
                        self.state = 'idle'
                    else:
                        self.publish_speed(self.config['undock_speed'])


class ChargingRos:
    """Connect the coordinator to the existing web Bridge node.
    将回充协调器接入现有网页 Bridge 节点。
    """

    def __init__(self, node, runner, license_monitor, nav, map_id, guard_ok, config,
                 lock=None):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String
        from std_srvs.srv import SetBool, Trigger
        from .. import qos
        from . import chassis

        self.node = node
        self.Twist, self.Bool, self.String = Twist, Bool, String
        self.SetBool = SetBool
        self.parse_chassis = chassis.parse_chassis_status
        self.dock_client = node.create_client(SetBool, '/chassis/set_auto_dock')
        self.hold_pub = node.create_publisher(Bool, '/charging/nav_hold', qos.status_qos())
        self.cmd_pub = node.create_publisher(Twist, '/charging/cmd_vel', qos.status_qos())
        self.status_pub = node.create_publisher(String, '/charging/status', qos.status_qos())
        self.coordinator = ChargingCoordinator(
            config, runner, license_monitor, nav, map_id, self._send_dock,
            self._publish_hold, self._publish_speed, guard_ok, lock=lock)
        node.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, qos.status_qos())
        node.create_subscription(DiagnosticStatus, '/chassis/dock_status', self._on_dock, qos.status_qos())
        node.create_subscription(Bool, '/battery/full', self._on_full, qos.status_qos())
        node.create_subscription(Odometry, '/odom', self._on_odom, qos.sensor_qos())
        node.create_service(Trigger, '/charging/request', self._on_request)
        node.create_timer(0.1, self._tick)

    def _send_dock(self, enable):
        if not self.dock_client.service_is_ready():
            self.coordinator.on_dock_result(False)
            return
        req = self.SetBool.Request()
        req.data = enable
        future = self.dock_client.call_async(req)
        future.add_done_callback(self._dock_result)

    def _dock_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.node.get_logger().error('回充服务失败: %s' % exc)
            self.coordinator.on_dock_result(False)
            return
        self.coordinator.on_dock_result(bool(result.success))

    def _publish_hold(self, hold):
        self.hold_pub.publish(self.Bool(data=hold))

    def _publish_speed(self, speed):
        msg = self.Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _on_diagnostics(self, msg):
        chassis = self.parse_chassis(msg)
        if chassis is not None:
            voltage = chassis.get('voltage_mv')
            self.coordinator.on_voltage(None if voltage is None else voltage / 1000.0)

    def _on_dock(self, msg):
        values = {item.key: item.value for item in msg.values}
        try:
            flags = [values[name] for name in ('charging', 'allow_recharge', 'dock_hw_present')]
            if any(value not in ('true', 'false') for value in flags):
                raise ValueError('无效布尔字段')
            current = int(values['charging_current_ma'])
        except (KeyError, TypeError, ValueError) as exc:
            self.node.get_logger().error('回充遥测无效: %s' % exc)
            self.coordinator.invalidate_dock()
            return
        self.coordinator.on_dock(flags[0] == 'true', flags[1] == 'true',
                                 current, flags[2] == 'true')

    def _on_full(self, msg):
        self.coordinator.on_full(msg.data)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.coordinator.on_odom(pose.position.x, pose.position.y, yaw)

    def _on_request(self, _request, response):
        response.success, response.message = self.coordinator.request()
        return response

    def _tick(self):
        self.coordinator.tick()
        self.status_pub.publish(self.String(data=json.dumps({
            'state': self.coordinator.state, 'reason': self.coordinator.reason,
            'distance_m': self.coordinator.distance}, ensure_ascii=False)))

    def on_nav_accepted(self, seq):
        self.coordinator.on_nav_accepted(seq)

    def on_nav_result(self, seq, status, reason):
        self.coordinator.on_nav_result(seq, status, reason)

    def abort(self, reason):
        self.coordinator.abort(reason)

    def operator_rearm(self):
        self.coordinator.operator_rearm()


def attach(node, runner, license_monitor, nav, map_id, guard_ok, lock=None):
    """Attach only when explicitly enabled; require every site parameter.
    仅显式启用时接线，要求提供全部现场参数。
    """
    node.declare_parameter('enable_autonomous_charging', False)
    if not node.get_parameter('enable_autonomous_charging').value:
        return None
    config = {}
    for key, name in PARAMETERS.items():
        node.declare_parameter(name, float('nan'))
        config[key] = node.get_parameter(name).value
    validate_config(config)
    return ChargingRos(node, runner, license_monitor, nav, map_id, guard_ok, config, lock)
