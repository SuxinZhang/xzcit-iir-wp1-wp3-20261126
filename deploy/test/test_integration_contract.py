import unittest
from pathlib import Path

DEPLOY = Path(__file__).resolve().parents[1]


class IntegrationContractTest(unittest.TestCase):
    """Verify the single HTTPS entry point and service dependency contract.
    验证单一 HTTPS 入口与 systemd 服务依赖契约。
    """

    def test_caddy_proxies_all_public_prefixes(self):
        text = (DEPLOY / 'Caddyfile').read_text(encoding='utf-8')
        for token in ('/api/inspection/*', '/ws/inspection*', '/captures/*',
                      '/whep/*', '/whip/*', '127.0.0.1:8089', '127.0.0.1:8889',
                      '127.0.0.1:8088'):
            self.assertIn(token, text)

    def test_target_includes_gateway_and_vision(self):
        target = (DEPLOY / 'systemd/iir.target').read_text(encoding='utf-8')
        self.assertIn('iir-gateway.service', target)
        self.assertIn('iir-vision-rknn.service', target)

    def test_new_services_exist_and_are_loopback_safe(self):
        gateway = (DEPLOY / 'systemd/iir-gateway.service').read_text(encoding='utf-8')
        vision = (DEPLOY / 'systemd/iir-vision-rknn.service').read_text(encoding='utf-8')
        self.assertIn('OPI_GATEWAY_HOST=127.0.0.1', gateway)
        self.assertIn('OPI_GATEWAY_BACKEND=ros', gateway)
        self.assertIn('run-vision.sh', vision)

    def test_port_map_covers_control_inspection_and_media(self):
        text = (DEPLOY / 'PORT_MAP.md').read_text(encoding='utf-8')
        for token in ('8088', '8089', '8889', '8095', '/api/inspection', '/ws/inspection'):
            self.assertIn(token, text)


if __name__ == '__main__':
    unittest.main()
