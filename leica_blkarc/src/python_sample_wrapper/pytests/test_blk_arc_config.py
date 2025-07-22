import re

import blk_arc_sample_wrapper.blk_arc_config as config


def test_connection_type():
    assert len(config.ConnectionType) == 2
    assert isinstance(config.ConnectionType.WIRED, config.ConnectionType)
    assert isinstance(config.ConnectionType.WIRELESS, config.ConnectionType)


def test_blk_arc_ip():
    assert len(config.BLK_ARC_IP) == 2
    assert config.ConnectionType.WIRED in config.BLK_ARC_IP
    assert config.ConnectionType.WIRELESS in config.BLK_ARC_IP
    assert type(config.BLK_ARC_IP[config.ConnectionType.WIRED]) == str
    assert type(config.BLK_ARC_IP[config.ConnectionType.WIRELESS]) == str
    ip_pattern = r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$"
    assert re.match(ip_pattern, config.BLK_ARC_IP[config.ConnectionType.WIRED])
    assert re.match(ip_pattern, config.BLK_ARC_IP[config.ConnectionType.WIRELESS])


def test_grpc_ports():
    assert type(config.BLK_ARC_GRPC_PORT) == int
    assert config.BLK_ARC_GRPC_PORT > 0
    assert type(config.GRPC_SECURE_PORT) == int
    assert config.GRPC_SECURE_PORT > 0


def test_tls_cert_endpoint():
    assert type(config.TLS_CERT_ENDPOINT) == str
