"""Non-blocking patrol-to-inspection gateway hook.
巡逻航点到点后触发巡检网关抓拍的非阻塞钩子。

The hook posts to the inspection Gateway REST endpoint in a daemon thread.
The Gateway then calls `/inspection/capture`, archives the images/JSON and
broadcasts the result on `/ws/inspection` to the management page and `/student`.
钩子以守护线程 POST 到巡检 Gateway；Gateway 再调用 `/inspection/capture`，
归档原图/标注图/JSON，并通过 `/ws/inspection` 推送到管理端与 `/student`。

A missing Gateway or vision service only logs a warning; the patrol state
machine is never blocked.
Gateway 或视觉服务离线只记录告警，巡逻状态机永不阻塞。
"""

from __future__ import annotations

import json
import os
import threading
import urllib.error
import urllib.request

DEFAULT_GATEWAY_URL = 'http://127.0.0.1:8089'


def build_capture_hook(node=None, gateway_url: str | None = None,
                       inspection_point_prefix: str = 'waypoint'):
    """Build a non-blocking ``(index, point) -> None`` gateway capture hook.
    构建非阻塞的 ``(index, point) -> None`` 网关抓拍钩子。

    Args:
        node: Optional ROS node used only for logging.
            可选 ROS 节点，仅用于记录日志。
        gateway_url: Gateway origin; defaults to ``OPI_GATEWAY_URL`` or loopback.
            Gateway 来源地址；默认取 ``OPI_GATEWAY_URL`` 或回环地址。
        inspection_point_prefix: Prefix used to name the patrol point.
            用于命名巡检点的前缀。
    """
    base = (gateway_url or os.environ.get('OPI_GATEWAY_URL', DEFAULT_GATEWAY_URL)).rstrip('/')
    logger = getattr(node, 'get_logger', None)

    def _warn(message: str) -> None:
        if logger is not None:
            try:
                logger().warning(message)
            except Exception:  # noqa: BLE001
                pass

    def trigger(index: int, point: dict) -> None:
        """Post one capture request without blocking the patrol thread.
        在不阻塞巡逻线程的情况下提交一次抓拍请求。
        """
        payload = json.dumps({
            'inspection_point': f'{inspection_point_prefix}-{index + 1}',
            'source': 'patrol',
            'archive': True,
        }).encode('utf-8')

        def _post() -> None:
            request = urllib.request.Request(
                base + '/api/inspection/capture',
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            try:
                with urllib.request.urlopen(request, timeout=8.0) as response:
                    body = json.load(response)
                if not body.get('capture_id'):
                    _warn('inspection capture returned no capture_id')
            except (urllib.error.URLError, TimeoutError, ValueError, OSError) as exc:
                _warn(f'inspection gateway unavailable: {exc}')

        threading.Thread(target=_post, name='iir-patrol-capture', daemon=True).start()

    return trigger


__all__ = ['build_capture_hook']
