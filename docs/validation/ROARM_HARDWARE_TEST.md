# RoArm-M2-S ROS1 Noetic Hardware Acceptance

Branch: `feat/roarm-control-kinematics-vla`

This procedure deliberately starts read-only and unlocks motion only after state, mapping and FK/IK checks pass. Stop at the first explicit error or unexpected mechanical motion.

## 0. Physical preparation

- Clear the arm workspace.
- Keep a hand on the physical power/stop method available to you.
- Do not place fingers near joints or the gripper.
- Do not manually add DTR/RTS toggles.
- A ROS `/roarm/soft_stop` is a software command gate, **not** an electrical emergency stop.

## 1. Synchronize safely

```bash
cd ~/xzb
git status --short
git fetch origin
```

If tracked files are modified, stop and preserve them before switching branches. Existing unrelated untracked `catkin_ws/src/vendor/` should not be deleted or staged.

Then:

```bash
git switch feat/roarm-control-kinematics-vla 2>/dev/null || \
git switch -c feat/roarm-control-kinematics-vla --track origin/feat/roarm-control-kinematics-vla
git pull --ff-only
git status --short
```

## 2. Verify USB identity

```bash
ls -l /dev/ttyUSB0
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_VENDOR_ID=|ID_MODEL_ID='
sudo lsof /dev/ttyUSB0
```

Expected CP2102N VID:PID is `10c4:ea60`; no other process should own the port.

## 3. Install dependencies

```bash
source /opt/ros/noetic/setup.bash
cd ~/xzb/catkin_ws
rosdep install --from-paths \
  src/xzb/xzb_interfaces \
  src/xzb/xzb_roarm_driver \
  src/xzb/xzb_description \
  src/xzb/xzb_roarm_control \
  src/xzb/xzb_roarm_moveit \
  src/xzb/xzb_runtime_bridge \
  --ignore-src -r -y
```

Stop on any rosdep error.

## 4. Build

```bash
catkin build xzb_interfaces xzb_roarm_driver xzb_description xzb_roarm_control xzb_roarm_moveit xzb_runtime_bridge
```

Do not proceed unless all listed packages succeed.

```bash
source ~/xzb/catkin_ws/devel/setup.bash
```

## 5. Run pure-Python tests

```bash
cd ~/xzb/catkin_ws/src/xzb/xzb_roarm_driver
PYTHONPATH=src python3 -m unittest discover -s test -p 'test_*.py' -v

cd ~/xzb/catkin_ws/src/xzb/xzb_roarm_control
PYTHONPATH=src python3 -m unittest discover -s test -p 'test_*.py' -v

cd ~/xzb/catkin_ws/src/xzb/xzb_runtime_bridge
PYTHONPATH=src python3 -m unittest discover -s test -p 'test_*.py' -v
```

Each suite must end with `OK`.

## 6. Read-only driver first

Terminal A:

```bash
source /opt/ros/noetic/setup.bash
source ~/xzb/catkin_ws/devel/setup.bash
roslaunch xzb_roarm_driver roarm_driver.launch port:=/dev/ttyUSB0 query_rate:=5.0 command_enable:=false
```

The arm must not receive motion commands from this node.

Terminal B:

```bash
source ~/xzb/catkin_ws/devel/setup.bash
rostopic echo -n 1 /roarm/connected
rostopic echo -n 1 /roarm/controller_joint_states
rostopic echo -n 1 /roarm/position
rostopic hz /roarm/controller_joint_states
```

Expect connected `True`, four controller positions and approximately 5 Hz state updates.

Then inspect mixed firmware/protocol traffic:

```bash
rostopic echo -n 10 /roarm/raw_response
```

`{"T":105}`, `T=1051` and occasional firmware text are acceptable.

If the arm moves unexpectedly during this read-only launch, stop immediately and report the complete launch log.

## 7. Full stack, still motion-disabled

Stop the read-only launch first. Then:

```bash
roslaunch xzb_roarm_control roarm_stack.launch port:=/dev/ttyUSB0 query_rate:=5.0
```

No movement is expected. Check:

```bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /roarm/tool_pose
rostopic echo -n 1 /roarm/status
rosservice list | grep '^/roarm/'
```

`/joint_states` must use:

- `base_link_to_link1`
- `link1_to_link2`
- `link2_to_link3`
- `link3_to_gripper_link`

The current real feedback may place shoulder slightly outside the nominal official command limit; telemetry allows a 0.1-rad margin, but commands remain strictly limited.

## 8. FK / IK sanity without movement

Copy the first three values from `/joint_states`, then:

```bash
rosrun xzb_roarm_control arm_test_cli.py fk <base> <shoulder> <elbow>
```

Copy XYZ from `/roarm/tool_pose`, then solve it with the same/current joint seed:

```bash
rosrun xzb_roarm_control arm_test_cli.py ik <x> <y> <z> --seed <base> <shoulder> <elbow>
```

Expected: IK succeeds with small residual. Controller `/roarm/position` does not have to numerically equal `/roarm/tool_pose` because their origins are currently different.

## 9. Explicitly enable bounded motion

Only after steps 1-8 pass:

```bash
rosrun xzb_roarm_control arm_test_cli.py enable
```

Expected service result: success true.

## 10. First joint motion — base only

Command only +0.03 rad:

```bash
rosrun xzb_roarm_control arm_test_cli.py joint-delta --base 0.03
```

Observe motion and `/joint_states`. Then return:

```bash
rosrun xzb_roarm_control arm_test_cli.py joint-delta --base -0.03
```

If direction, distance or feedback is unexpected:

```bash
rosrun xzb_roarm_control arm_test_cli.py soft-stop
```

Then stop testing and report logs/state. Do not compensate with a larger opposite command.

## 11. Shoulder and elbow

Test one joint at a time, +0.03 then -0.03:

```bash
rosrun xzb_roarm_control arm_test_cli.py joint-delta --shoulder 0.03
rosrun xzb_roarm_control arm_test_cli.py joint-delta --shoulder -0.03
rosrun xzb_roarm_control arm_test_cli.py joint-delta --elbow 0.03
rosrun xzb_roarm_control arm_test_cli.py joint-delta --elbow -0.03
```

## 12. Gripper first motion

Do **not** begin with normalized full open/close. First verify sign and feedback with a 0.03-rad model-joint delta:

```bash
rosrun xzb_roarm_control arm_test_cli.py joint-delta --gripper 0.03
rosrun xzb_roarm_control arm_test_cli.py joint-delta --gripper -0.03
```

After this direction/range is confirmed we can tune the semantic `gripper 0..1` endpoints.

## 13. Cartesian 5 mm test

Only after individual joints pass:

```bash
rosrun xzb_roarm_control arm_test_cli.py cart-delta --x 0.005
rosrun xzb_roarm_control arm_test_cli.py cart-delta --x -0.005
```

The request is limited both in Cartesian space and again after IK in joint space, so a near-singular IK result requiring a large hidden joint jump is rejected.

Repeat Y/Z only after X is correct:

```bash
rosrun xzb_roarm_control arm_test_cli.py cart-delta --y 0.005
rosrun xzb_roarm_control arm_test_cli.py cart-delta --y -0.005
rosrun xzb_roarm_control arm_test_cli.py cart-delta --z 0.005
rosrun xzb_roarm_control arm_test_cli.py cart-delta --z -0.005
```

## 14. Disconnect recovery

Disable motion first:

```bash
rosrun xzb_roarm_control arm_test_cli.py disable
```

Unplug USB. `/roarm/connected` must become false. Reconnect and wait for fresh state. Motion must remain disabled until `enable` is explicitly called again.

If the device number changes:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

Report the output; a permanent `/dev/roarm` rule can then be installed.

## 15. MoveIt1 offline smoke test

Keep hardware trajectory execution disabled:

```bash
roslaunch xzb_roarm_moveit demo.launch
```

Verify `move_group` starts and the `hand` group/KDL solver load without fatal errors. Do not enable `allow_trajectory_execution` during first hardware acceptance.

The optional action bridge `/roarm_arm_controller/follow_joint_trajectory` is disabled by default in the control launch and every waypoint still passes the 0.05-rad joint-step guard.

## 16. VLA mode-C dry-run

```bash
roslaunch xzb_runtime_bridge vla_arm_adapter.launch dry_run:=true hardware_enable:=false allow_joint_mode:=false
```

The adapter should report dry-run mode. Publishing a valid Cartesian `ArmAction` should appear on `/vla/arm_action_checked` but must not move the arm.

Real VLA hardware execution remains prohibited until deterministic arm control, hand-eye calibration, camera transform, TCP offset and grasp safety are accepted.

## Feedback to return

Send these results back for the next optimization pass:

1. catkin build summary
2. all three unittest summaries
3. read-only launch log
4. one `/roarm/controller_joint_states` sample
5. one `/joint_states` sample
6. one `/roarm/position` and `/roarm/tool_pose` sample
7. FK/IK service results
8. base/shoulder/elbow/gripper +0.03/-0.03 observed directions
9. X/Y/Z 5 mm test results or first rejection/error
10. disconnect/reconnect result
11. MoveIt1 startup log
12. VLA dry-run status
