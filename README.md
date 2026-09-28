# Checkpoint 16 — Mobile Robot Kinematics

A ROSBot XL mecanum-wheel learning project targeting Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress. The learner implements the core code incrementally, with scaffolding, review, and verification support from the coach. Simulation dependencies are managed separately; original course materials are not included.

## Current status

- Task 1: wheel-speed publishing, forward kinematics, and a launch file for both nodes are implemented. Local message checks and simulation odometry checks have passed.
- Task 2: `eight_trajectory` is not implemented yet.
- Independent Gazebo model-pose checks and final validation in the course cloud environment remain pending. The full checkpoint is not complete.

## Data flow and conventions

`wheel_velocities_publisher` → `/wheel_speed` → `kinematic_model` → `/cmd_vel` → simulation controller

- `/wheel_speed`: `std_msgs/msg/Float32MultiArray`, ordered `[FL, FR, RL, RR]` (front left, front right, rear left, rear right), in rad/s.
- `/cmd_vel`: `geometry_msgs/msg/Twist`. In the body frame, x points forward, y points left, and positive rotation about z is counterclockwise when viewed from above.
- The publisher checks its motion stage every 100 ms. It commands forward, backward, left, right, clockwise, and counterclockwise motion for approximately 3 seconds each, then continuously publishes zero wheel speeds. Timer resolution and scheduling affect transition times.
- Translation commands have magnitude 0.1 m/s; rotation commands have magnitude 0.5 rad/s. This is a timed sequence without closed-loop odometry correction.
- The model rejects inputs whose length is not four or that contain non-finite values. No input-timeout stop is implemented; rejecting an input does not actively publish a zero command.

## Geometry and model

Wheel radius `r = 0.05 m`; front-to-rear wheel-center distance `0.17 m`; left-to-right wheel-center distance `0.27 m`. The corresponding half-distances are `lx = 0.085 m` and `ly = 0.135 m`, so `k = lx + ly = 0.22 m`.

```text
vx = r/4     * ( w_FL + w_FR + w_RL + w_RR)
vy = r/4     * (-w_FL + w_FR + w_RL - w_RR)
Ω  = r/(4*k) * (-w_FL + w_FR - w_RL + w_RR)
```

These three geometry parameters were checked against both the local and course cloud configurations. The drawing specifies a lateral wheel-center distance of 269.69 mm, slightly different from the configured 270 mm used by the code.

## Build and run

On Ubuntu 22.04 with ROS 2 Humble and colcon configured, place this repository at `~/ros2_ws/src/checkpoint16`:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select wheel_velocities_publisher kinematic_model
source install/setup.bash
ros2 launch kinematic_model kinematic_model.launch.py
```

The launch file starts only the two assignment nodes. To inspect messages, use another terminal with the same environment sourced:

```bash
ros2 topic echo /wheel_speed
ros2 topic echo /cmd_vel
```

For Gazebo integration, first start a compatible ROSBot XL mecanum-wheel simulation whose controller subscribes to `/cmd_vel`. The assignment and simulation processes must use matching `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` settings. Avoid running other velocity publishers simultaneously.

Local validation used a separate `checkpoint16_sim_ws` workspace with `ROS_DOMAIN_ID=116` and `ROS_LOCALHOST_ONLY=1`. Simulation dependencies and adaptation scripts are not distributed with this repository. Feedback was read from `/rosbot_xl_base_controller/odom`, and output on `/odometry/filtered` was also checked. Other course versions may require different configuration.

## Validation performed (2026-09-29)

| Check | Result |
| --- | --- |
| Build both C++ packages | Passed |
| Nine model inputs: six directions, two mixed motions, and zero speed | All Twist components matched |
| Independent prediction and runtime check for input `[1,3,3,1]` | `(0.1 m/s, 0.05 m/s, 0 rad/s)` |
| Launch both nodes in isolated domain 117 | 213 wheel-speed messages and 213 Twist messages; correct six-direction and stop sequence |
| Gazebo integration in domain 116 | Odometry translation approximately ±0.1 m/s, rotation approximately ±0.5 rad/s, and final velocity approximately zero; all seven stages passed |

Validation used coach-provided helper scripts outside this repository. Detailed training records and raw logs remain local. This repository does not yet include a directly runnable automated test suite. Matching controller odometry does not constitute independent model-pose verification.

## Next steps

- Check independent simulation poses and validate interfaces and execution in the course cloud environment.
- Incrementally implement `eight_trajectory`, pose feedback, and trajectory validation.
