import socket
import time
import unittest


class FakeRunCommandDds:
    def __init__(self):
        self.commands = []

    def write_run_command(self, command):
        self.commands.append(command)


class G1NavUdpCmdBridgeTests(unittest.TestCase):
    def test_parse_json_payload_command(self):
        from ros2_bridge.g1_nav_cmd_udp_bridge import parse_udp_command_payload

        self.assertEqual(
            parse_udp_command_payload(b'{"payload": "[0.1, 0.0, -0.2, 0.8]"}'),
            [0.1, 0.0, -0.2, 0.8],
        )

    def test_bridge_writes_received_command_to_run_command_dds(self):
        from ros2_bridge.g1_nav_cmd_udp_bridge import (
            G1NavUdpCmdBridge,
            G1NavUdpCmdBridgeConfig,
        )

        fake_dds = FakeRunCommandDds()
        bridge = G1NavUdpCmdBridge(
            fake_dds,
            G1NavUdpCmdBridgeConfig(host="127.0.0.1", port=0),
        )
        bridge.start()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(
                    b'{"payload": "[0.2, 0.0, -0.3, 0.8]"}',
                    ("127.0.0.1", bridge.port),
                )
            deadline = time.time() + 1.0
            while time.time() < deadline and not fake_dds.commands:
                time.sleep(0.01)
        finally:
            bridge.stop()

        self.assertEqual(fake_dds.commands[-1], [0.2, 0.0, -0.3, 0.8])


if __name__ == "__main__":
    unittest.main()
