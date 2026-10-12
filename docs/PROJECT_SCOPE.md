# Xzb Robot Project Scope

## Runtime

- Ubuntu 20.04
- ROS1 Noetic
- Catkin
- No ROS2 runtime in this project

## Hardware

- Bingda four-wheel four-steering chassis
- RoArm-M2-S
- Intel RealSense D435i mounted Eye-in-Hand
- Linear actuator inspection mast
- PAN/TILT ST3215 servos
- Inspection camera on mast

## Existing source code that must be reused

1. Bingda ROS1 Noetic `spider` source
2. RoArm-M2-S Python source supplied by the project owner
3. `linear_actuator_control` ROS1 source supplied by the project owner
4. `door_detection` source supplied by the project owner
5. Existing Orbbec ROS1 camera stack from the chassis project where applicable
6. RealSense ROS1 SDK for D435i

## Architecture Rules

- Existing working hardware drivers are reused, not rewritten without reason.
- Vendor source is kept separate from Xzb business logic.
- Agent / VLM / VLA cannot directly control motors, PWM, serial devices or raw joints.
- All AI actions pass through ROS1 skills and an independent safety layer.
- D435i is the RoArm Eye-in-Hand camera.
- Inspection mast camera performs door/window state inspection and long-range trash search.
- UI supports Chinese and English.
- First VLA integration target is LeRobot + SmolVLA.
- GR00T remains a later optional backend.
