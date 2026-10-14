import unittest
import numpy as np
from xzb_roarm_control.joint_mapping import controller_to_ros, ros_to_controller
from xzb_roarm_control.safety import ArmSafety, SafetyError
from xzb_roarm_control.kinematics import RoArmKinematics
from xzb_roarm_control.control_core import ArmControlCore, ControlRejected

class CoreTests(unittest.TestCase):
    def test_mapping_round_trip(self):
        x=(-.2,.3,1.1,2.4); self.assertTrue(np.allclose(ros_to_controller(controller_to_ros(x)),x))
    def test_real_state_margin_does_not_relax_command_limit(self):
        s=ArmSafety(); s.validate_joint_state((.007,-1.5923,.03,1.44))
        with self.assertRaises(SafetyError): s.validate_joint_target((.007,-1.5923,.03,1.44))
    def test_fk_matches_official_ikfast_reference(self):
        p=RoArmKinematics().forward((.25,-.35,1.2)).position
        ref=(.3874571057679583,-.09638062256532237,.3390563301496208)
        self.assertLess(np.linalg.norm(np.array(p)-np.array(ref)),1e-8)
    def test_ik_round_trip(self):
        k=RoArmKinematics(); p=k.forward((.25,-.35,1.2)).position; r=k.inverse(p,(.2,-.3,1.1))
        self.assertTrue(r.success); self.assertLess(r.residual,1e-4)
    def test_absolute_large_jump_rejected(self):
        now=[0.0]; c=ArmControlCore(lambda t,s:True,clock=lambda:now[0]); c.set_connected(True); c.update_joint_state((0,0,1.5,.7)); c.set_motion_enabled(True)
        with self.assertRaises(SafetyError): c.move_joints((.2,0,1.5,.7),relative=False)
    def test_single_joint_delta_uses_single_dispatcher(self):
        full=[]; single=[]; now=[0.0]
        c=ArmControlCore(lambda t,s:full.append(t) or True,single_dispatcher=lambda i,t,s:single.append((i,t,s)) or True,clock=lambda:now[0])
        c.set_connected(True); c.update_joint_state((0,0,1.5,.7)); c.set_motion_enabled(True)
        result=c.move_joints((.03,0,0,0),True)
        self.assertTrue(result.accepted); self.assertEqual(full,[]); self.assertEqual(single,[(0,.03,.1)])
    def test_multi_joint_delta_keeps_full_dispatcher(self):
        full=[]; single=[]; now=[0.0]
        c=ArmControlCore(lambda t,s:full.append(t) or True,single_dispatcher=lambda i,t,s:single.append((i,t,s)) or True,clock=lambda:now[0])
        c.set_connected(True); c.update_joint_state((0,0,1.5,.7)); c.set_motion_enabled(True)
        result=c.move_joints((.01,.01,0,0),True)
        self.assertTrue(result.accepted); self.assertEqual(single,[]); self.assertEqual(full[-1],(.01,.01,1.5,.7))
    def test_stale_state_rejects_motion(self):
        now=[0.0]; c=ArmControlCore(lambda t,s:True,state_timeout=1,clock=lambda:now[0]); c.set_connected(True); c.update_joint_state((0,0,1.5,.7)); c.set_motion_enabled(True); now[0]=2
        with self.assertRaises(ControlRejected): c.move_joints((.01,0,0,0),True)
    def test_oversized_cartesian_delta_rejected(self):
        now=[0.0]; c=ArmControlCore(lambda t,s:True,clock=lambda:now[0]); c.set_connected(True); c.update_joint_state((0,0,1.5,.7)); c.set_motion_enabled(True)
        with self.assertRaises(SafetyError): c.move_cartesian((.02,0,0),True)

if __name__ == '__main__': unittest.main()
