import unittest

from xzb_roarm_driver.protocol import classify_line, make_boot_mission_query, make_state_query


class ProtocolTests(unittest.TestCase):
    def test_make_state_query_is_newline_delimited_json(self):
        self.assertEqual(make_state_query(), b'{"T":105}\n')

    def test_make_boot_mission_query_is_exact_read_only_t602(self):
        self.assertEqual(make_boot_mission_query(), b'{"T":602}\n')

    def test_classifies_t1051_state(self):
        line = '{"T":1051,"x":515.8803852,"y":-3.956830589,"z":-49.68354938,"b":-0.007669904,"s":1.592272058,"e":0.030679616,"t":1.695048771,"torB":36,"torS":104,"torE":92,"torH":300}'
        event = classify_line(line)
        self.assertEqual(event.kind, 'state')
        self.assertEqual(event.message_type, 1051)
        self.assertAlmostEqual(event.data['x_m'], 0.5158803852)
        self.assertAlmostEqual(event.data['y_m'], -0.003956830589)
        self.assertAlmostEqual(event.data['z_m'], -0.04968354938)
        self.assertAlmostEqual(event.data['base_rad'], -0.007669904)
        self.assertAlmostEqual(event.data['shoulder_rad'], 1.592272058)
        self.assertAlmostEqual(event.data['elbow_rad'], 0.030679616)
        self.assertAlmostEqual(event.data['hand_rad'], 1.695048771)
        self.assertEqual(event.data['torB'], 36)
        self.assertEqual(event.data['torH'], 300)

    def test_classifies_command_echo_without_treating_it_as_state(self):
        event = classify_line('{"T":105}')
        self.assertEqual(event.kind, 'protocol')
        self.assertEqual(event.message_type, 105)
        self.assertEqual(event.data, {'T': 105})

    def test_non_json_firmware_line_is_log_not_error(self):
        event = classify_line('Bytes received: 214')
        self.assertEqual(event.kind, 'firmware_log')
        self.assertIsNone(event.message_type)
        self.assertEqual(event.raw, 'Bytes received: 214')

    def test_malformed_state_json_is_protocol_message_not_state(self):
        event = classify_line('{"T":1051,"x":1}')
        self.assertEqual(event.kind, 'invalid_state')
        self.assertEqual(event.message_type, 1051)
        self.assertIn('missing', event.error)

    def test_non_numeric_state_is_invalid_state_not_exception(self):
        line = '{"T":1051,"x":"bad","y":0,"z":0,"b":0,"s":0,"e":0,"t":0,"torB":0,"torS":0,"torE":0,"torH":0}'
        event = classify_line(line)
        self.assertEqual(event.kind, 'invalid_state')
        self.assertEqual(event.message_type, 1051)
        self.assertIn('numeric', event.error)

    def test_blank_line_is_ignored(self):
        event = classify_line('   ')
        self.assertEqual(event.kind, 'empty')


if __name__ == '__main__':
    unittest.main()
