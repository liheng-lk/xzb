import unittest
from xzb_runtime_bridge.vla_arm_adapter import ArmAction, CARTESIAN_DELTA, VLAActionRejected, VLAArmAdapter
from xzb_runtime_bridge.smolvla_adapter import SmolVLAActionMapper

class VLAAdapterTests(unittest.TestCase):
    def test_dry_run_never_dispatches(self):
        calls=[]; a=VLAArmAdapter(lambda x:calls.append(x) or True)
        _,sent=a.process(ArmAction.cartesian_delta(.005,0,0,.5)); self.assertFalse(sent); self.assertEqual(calls,[])
    def test_hardware_requires_both_gates(self):
        calls=[]; action=ArmAction.cartesian_delta(.005,0,0,.5)
        for dry,hw,expected in [(True,True,False),(False,False,False),(False,True,True)]:
            a=VLAArmAdapter(lambda x:calls.append(x) or True,dry_run=dry,hardware_enable=hw); _,sent=a.process(action); self.assertEqual(sent,expected)
    def test_oversized_delta_rejected(self):
        with self.assertRaises(VLAActionRejected): VLAArmAdapter().process(ArmAction.cartesian_delta(.02,0,0,.5))
    def test_joint_mode_default_disabled(self):
        with self.assertRaises(VLAActionRejected): VLAArmAdapter().process(ArmAction.joint(0,0,1,.5))
    def test_smol_mapping(self):
        a=SmolVLAActionMapper().map([.001,.002,.003,.4]); self.assertEqual(a.mode,CARTESIAN_DELTA); self.assertEqual(a.gripper,.4)

if __name__ == '__main__': unittest.main()
