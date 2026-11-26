"""Bind infrared permission to the most recently received target and session.
将红外运动许可绑定到最近收到的目标与会话。
"""
import json
import math


class DockSafety:
    def __init__(self):
        self.target = None
        self.permission = None

    def reset(self):
        self.target = None
        self.permission = None

    def on_target(self, target, now, session):
        if self.target is None or self.target[0] != target or self.target[2] != session:
            self.permission = None
        self.target = (target, float(now), session)

    def on_permission(self, raw, now):
        try:
            payload = json.loads(raw)
            dock = payload['dock']
            allowed = payload['allowed']
            generation = dock['generation']
            session = dock['session']
            ttl = payload['ttl_s']
            valid = (payload['v'] == 1 and type(allowed) is bool and
                     dock['active'] is True and type(generation) is int and
                     0 <= generation <= 0xffffffff and type(session) is int and
                     0 <= session <= 0xffffffff and type(ttl) in (int, float) and
                     math.isfinite(ttl) and ttl > 0)
        except (ValueError, TypeError, KeyError):
            valid = False
        if not valid or not allowed:
            self.permission = None
            return
        self.permission = (generation, session, float(now) + min(float(ttl), 0.3))

    def permitted_generation(self, now):
        if self.target is None or self.permission is None:
            return None
        target, received, target_session = self.target
        generation, session, expires = self.permission
        if (not target.active or now < received or now - received > 0.3 or
                now > expires or generation != target.generation or session != target_session):
            return None
        return generation
