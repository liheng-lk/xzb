import unittest

from xzb_roarm_driver.driver_core import ReadOnlyDriverCore, joint_state_values, pose_values


class FakeTransport:
    def __init__(self, connected=True):
        self.connected = connected
        self.writes = []

    def write(self, payload):
        self.writes.append(payload)


class DriverCoreTests(unittest.TestCase):
    def test_handle_state_line_emits_raw_then_state(self):
        transport = FakeTransport()
        raw = []
        states = []
        core = ReadOnlyDriverCore(transport, on_raw=raw.append, on_state=states.append)
        line = '{"T":1051,"x":100,"y":200,"z":-50,"b":0.1,"s":0.2,"e":0.3,"t":0.4,"torB":1,"torS":2,"torE":3,"torH":4}'

        event = core.handle_line(line)

        self.assertEqual(raw, [line])
        self.assertEqual(event.kind, 'state')
        self.assertEqual(len(states), 1)
        self.assertAlmostEqual(states[0]['x_m'], 0.1)

    def test_handle_firmware_log_emits_raw_but_not_state(self):
        transport = FakeTransport()
        raw = []
        states = []
        core = ReadOnlyDriverCore(transport, on_raw=raw.append, on_state=states.append)
        event = core.handle_line('Bytes received: 214')
        self.assertEqual(event.kind, 'firmware_log')
        self.assertEqual(raw, ['Bytes received: 214'])
        self.assertEqual(states, [])

    def test_query_state_sends_only_t105_when_connected(self):
        transport = FakeTransport(connected=True)
        core = ReadOnlyDriverCore(transport)
        self.assertTrue(core.query_state())
        self.assertEqual(transport.writes, [b'{"T":105}\n'])

    def test_query_state_does_nothing_when_disconnected(self):
        transport = FakeTransport(connected=False)
        core = ReadOnlyDriverCore(transport)
        self.assertFalse(core.query_state())
        self.assertEqual(transport.writes, [])

    def test_query_boot_mission_sends_only_t602_when_connected(self):
        transport = FakeTransport(connected=True)
        core = ReadOnlyDriverCore(transport)
        self.assertTrue(core.query_boot_mission())
        self.assertEqual(transport.writes, [b'{"T":602}\n'])

    def test_query_boot_mission_does_nothing_when_disconnected(self):
        transport = FakeTransport(connected=False)
        core = ReadOnlyDriverCore(transport)
        self.assertFalse(core.query_boot_mission())
        self.assertEqual(transport.writes, [])

    def test_cartesian_target_uses_vendor_t1041(self):
        transport = FakeTransport(connected=True)
        core = ReadOnlyDriverCore(transport)
        self.assertTrue(core.send_cartesian_target(235, 0, 234, 3.14))
        self.assertEqual(transport.writes, [b'{"T":1041,"x":235.0,"y":0.0,"z":234.0,"t":3.14}\n'])

    def test_cartesian_target_rejected_when_disconnected(self):
        transport = FakeTransport(connected=False)
        core = ReadOnlyDriverCore(transport)
        self.assertFalse(core.send_cartesian_target(235, 0, 234, 3.14))
        self.assertEqual(transport.writes, [])

    def test_boot_mission_capture_publishes_complete_read_only_dump(self):
        transport = FakeTransport(connected=True)
        captures = []
        core = ReadOnlyDriverCore(transport, on_boot_mission=captures.append)
        self.assertTrue(core.query_boot_mission())

        lines = [
            '{"T":602}',
            '---=== File Content ===---',
            'reading file: [boot] starts:',
            '',
            '{"name":"boot","intro":"these cmds run automatically at boot."}',
            '{"T":105}',
            '{"T":1051,"x":1,"y":2,"z":3,"b":0,"s":0,"e":0,"t":0,"torB":0,"torS":0,"torE":0,"torH":0}',
            '[StepNum: 1 ] - {"T":112,"mode":1,"b":50,"s":50,"e":50,"h":50}',
            '^^^ ^^^ ^^^ reading file: boot.mission ends. ^^^ ^^^ ^^^',
        ]
        for line in lines:
            core.handle_line(line)

        self.assertEqual(len(captures), 1)
        self.assertIn('---=== File Content ===---', captures[0])
        self.assertIn('[StepNum: 1 ] - {"T":112', captures[0])
        self.assertIn('reading file: boot.mission ends.', captures[0])
        self.assertNotIn('{"T":105}\n', captures[0])
        self.assertNotIn('{"T":1051', captures[0])

    def test_boot_mission_file_not_found_is_reported_only_for_pending_query(self):
        transport = FakeTransport(connected=True)
        captures = []
        core = ReadOnlyDriverCore(transport, on_boot_mission=captures.append)
        core.handle_line('file not found.')
        self.assertEqual(captures, [])
        self.assertTrue(core.query_boot_mission())
        core.handle_line('file not found.')
        self.assertEqual(captures, ['file not found.'])

    def test_joint_state_values_preserve_controller_radians(self):
        state = {'base_rad': -0.1, 'shoulder_rad': 1.2, 'elbow_rad': 0.3, 'hand_rad': 1.6}
        names, positions = joint_state_values(state)
        self.assertEqual(names, ['base', 'shoulder', 'elbow', 'hand'])
        self.assertEqual(positions, [-0.1, 1.2, 0.3, 1.6])

    def test_pose_values_are_position_only_with_identity_orientation(self):
        state = {'x_m': 0.5, 'y_m': -0.01, 'z_m': -0.05}
        self.assertEqual(pose_values(state), (0.5, -0.01, -0.05, 0.0, 0.0, 0.0, 1.0))


if __name__ == '__main__':
    unittest.main()
