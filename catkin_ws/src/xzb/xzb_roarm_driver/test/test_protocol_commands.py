import json
import math
import unittest
from xzb_roarm_driver.protocol import make_joint_command, make_single_joint_command

class ProtocolCommandTests(unittest.TestCase):
    def test_joint_command_exact_fields_and_newline(self):
        wire = make_joint_command(0.1, -0.2, 0.3, 2.8, speed=0, acceleration=10)
        self.assertTrue(wire.endswith(b'\n'))
        self.assertEqual(json.loads(wire.decode()), {"T":102,"base":0.1,"shoulder":-0.2,"elbow":0.3,"hand":2.8,"spd":0.0,"acc":10.0})

    def test_single_joint_command_uses_t101_and_only_one_joint(self):
        wire = make_single_joint_command(1, -0.03, speed=0, acceleration=10)
        self.assertTrue(wire.endswith(b'\n'))
        self.assertEqual(json.loads(wire.decode()), {"T":101,"joint":1,"rad":-0.03,"spd":0.0,"acc":10.0})

    def test_single_joint_command_rejects_invalid_joint(self):
        with self.assertRaises(ValueError): make_single_joint_command(0, 0.1)
        with self.assertRaises(ValueError): make_single_joint_command(5, 0.1)

    def test_joint_command_rejects_non_finite(self):
        with self.assertRaises(ValueError): make_joint_command(math.nan, 0, 0, 0)
        with self.assertRaises(ValueError): make_joint_command(0, 0, math.inf, 0)
        with self.assertRaises(ValueError): make_single_joint_command(1, math.nan)

if __name__ == '__main__': unittest.main()
