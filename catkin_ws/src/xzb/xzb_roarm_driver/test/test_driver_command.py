import unittest
from xzb_roarm_driver.driver_core import ReadOnlyDriverCore

class Transport:
    def __init__(self, connected=True): self.connected=connected; self.writes=[]
    def write(self,payload): self.writes.append(payload)

class DriverCommandTests(unittest.TestCase):
    def test_disconnected_joint_command_rejected(self):
        t=Transport(False); c=ReadOnlyDriverCore(t)
        self.assertFalse(c.send_joint_target(0,0,1,2.5)); self.assertEqual(t.writes,[])
    def test_joint_command_writes_t102(self):
        t=Transport(True); c=ReadOnlyDriverCore(t)
        self.assertTrue(c.send_joint_target(0.1,-0.2,0.3,2.8)); self.assertIn(b'"T":102',t.writes[0])

if __name__ == '__main__': unittest.main()
