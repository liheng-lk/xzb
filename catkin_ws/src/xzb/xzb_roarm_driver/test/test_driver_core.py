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
