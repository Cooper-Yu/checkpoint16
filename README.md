# Checkpoint 16 — Mobile Robot Kinematics

ROSBot XL mecanum-wheel exercises for Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress. Core implementations were written incrementally by the learner with coaching, scaffolding, and verification support. Course materials and simulator dependencies are not included.

## Status

- Task 1: timed wheel commands, forward kinematics, and two-node launch implemented and locally verified.
- Task 2: odometry feedback, eight-segment trajectory, waypoint transitions, timeout stopping, and two-node launch implemented and locally verified. A reduced-help coordinate-transform reconstruction also passed.
- Course cloud acceptance is pending. Local motion evidence uses controller odometry; independent Gazebo model-pose validation remains pending.

## Packages and interfaces

| Package | Responsibility |
| --- | --- |
| `wheel_velocities_publisher` | Task 1: forward, backward, left, right, clockwise, counterclockwise, then stop |
| `kinematic_model` | Convert four wheel velocities into body-frame Twist commands |
| `eight_trajectory` | Task 2: follow eight planned waypoints using odometry feedback |

```text
Task 1: wheel_velocities_publisher -> /wheel_speed -> kinematic_model -> /cmd_vel
Task 2: odometry -> eight_trajectory -> /wheel_speed -> kinematic_model -> /cmd_vel
```

- `/wheel_speed`: `std_msgs/msg/Float32MultiArray`, ordered `[FL, FR, RL, RR]`, in rad/s.
- `/cmd_vel`: `geometry_msgs/msg/Twist`; body x forward, y left, positive yaw counterclockwise.
- The Task 2 node uses `/odom` (`nav_msgs/msg/Odometry`) internally. The launch defaults to remapping it to `/rosbot_xl_base_controller/odom`, verified in both local and cloud simulations. The `odom_topic` argument can override this mapping.
- Odometry positions, goals, and world-frame velocities use the same fixed odometry frame. No TF conversion is performed.
- Run only one wheel-command source at a time. Do not run the Task 1 publisher alongside Task 2.

## Kinematics and control

The configured wheel radius is `r = 0.05 m`. Wheel-center distances are `0.17 m` front-to-rear and `0.27 m` left-to-right, giving `k = 0.085 + 0.135 = 0.22 m`. These parameters were checked against local and course cloud configurations. The drawing's lateral dimension is 269.69 mm; the code uses the configured 270 mm.

```text
vx = r/4     * ( w_FL + w_FR + w_RL + w_RR)
vy = r/4     * (-w_FL + w_FR + w_RL - w_RR)
omega = r/(4*k) * (-w_FL + w_FR - w_RL + w_RR)
```

Task 1 checks stages every 100 ms, commanding each motion for approximately 3 seconds. Translation magnitude is 0.1 m/s and rotation magnitude is 0.5 rad/s. It continuously publishes zero after the final stage.

Task 2 initializes its first goal from the first valid pose. Subsequent goals accumulate the configured world-frame increments from the previous planned goal, rather than from the actual stopping position. Translation and rotation can occur simultaneously.

| Setting | Value |
| --- | --- |
| Control period | 100 ms |
| Position tolerance | 0.02 m |
| Heading tolerance | 0.05 rad |
| Distance gain | 0.5 /s |
| Maximum linear speed | 0.1 m/s |
| Heading gain | 1.0 /s |
| Maximum angular speed | 0.5 rad/s |
| Feedback timeout | 0.5 s, steady wall time |

Linear speed is proportional to distance, capped at the maximum and zero inside tolerance. The desired world velocity is rotated into the body frame using the current yaw. Heading control uses the shortest wrapped error and symmetric angular speed limits. A waypoint completes only when both tolerances are satisfied. The controller sends zero during the transition cycle and continuously after all eight segments complete.

Missing or stale feedback causes repeated zero wheel commands. Fresh feedback resumes an unfinished trajectory. Feedback freshness measures local receipt time, not the message timestamp. Numeric pose/quaternion validation is performed; this is not a full estimator-quality check. The final heading changes by approximately -pi relative to the start; returning near the starting position does not mean returning to the starting orientation.

`kinematic_model` rejects malformed or non-finite wheel inputs but has no independent input-timeout stop. Task 2's timeout protection requires that node to remain running; downstream controller timeout behavior is environment-dependent.

## Build

Place the repository at `~/ros2_ws/src/checkpoint16`:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select wheel_velocities_publisher kinematic_model eight_trajectory
source install/setup.bash
```

ROS dependencies include `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `launch`, and `launch_ros`.

## Run

First start the compatible ROSBot XL mecanum simulation. Use matching ROS domain and localhost settings in all simulation and application terminals.

Task 1:

```bash
ros2 launch kinematic_model kinematic_model.launch.py
```

Task 2 with the verified local and cloud controller odometry (no argument needed):

```bash
ros2 launch eight_trajectory eight_trajectory.launch.py
```

For an environment that instead publishes `/odom`:

```bash
ros2 launch eight_trajectory eight_trajectory.launch.py \
  odom_topic:=/odom
```

Local simulation used a separate `checkpoint16_sim_ws` workspace and `ROS_DOMAIN_ID=116`, `ROS_LOCALHOST_ONLY=1`. Set these only when matching that local simulator; do not assume the cloud uses the same values. The launch files start assignment nodes, not Gazebo.

## Cloud acceptance checklist

Use the `codex/checkpoint16` branch. For a new checkout:

```bash
mkdir -p ~/ros2_ws/src
git clone --branch codex/checkpoint16 \
  https://github.com/Cooper-Yu/checkpoint16.git ~/ros2_ws/src/checkpoint16
```

For an existing checkout, inspect local changes before updating; do not overwrite uncommitted cloud work. Build and source the workspace as above, then:

1. Start the course simulation with mecanum wheels and check its actual odometry topic and `/cmd_vel` interface.
2. Stop any other wheel or velocity command publisher.
3. Start the Task 2 launch, overriding `odom_topic` only if required.
4. Observe all eight waypoints, the two rotation segments, and sustained stopping at the end. Confirm each position and heading against the course requirements.

Useful inspection commands, in a separately sourced terminal:

```bash
ros2 topic info /wheel_speed --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic echo /wheel_speed
ros2 topic echo /rosbot_xl_base_controller/odom
```

Use the selected feedback topic if you override the default. Cloud trajectory logs match the eight planned targets; sustained final-stop evidence and formal course acceptance remain pending.

## Local validation (2026-09-29 to 2026-09-30)

| Check | Result |
| --- | --- |
| Task 1 model inputs and timed sequence | Six directions, mixed inputs, zero command, and stage transitions passed |
| Task 1 Gazebo controller odometry | Approximately ±0.1 m/s translation, ±0.5 rad/s rotation, then stop |
| Task 2 helper and transition tests | Coordinate transforms, tolerances, planned goal accumulation, eight transitions, final boundary, and invalid-result branches passed |
| Task 2 controlled ROS input | Missing/stale feedback stopping, recovery, and final completion behavior passed |
| Full eight-segment Gazebo run | Eight waypoints entered both tolerances in order; final stable stop after about 136.4 s |
| Full-run final errors | Position 0.01850 m; shortest heading error 0.04753 rad |
| Task 2 launch | Two intended nodes, one publisher per command topic, odometry remapping, expected Twist output, and timeout stop passed |
| Reduced-help learner reconstruction | Six known-vector and four norm/round-trip coordinate-transform checks passed |

The full run started at yaw 0.02000 rad, so its planned final yaw was -3.12160 rad; measured final yaw was -3.07407 rad, within tolerance. Recorded waypoint position errors were below 0.02 m and heading errors below 0.05 rad.

Coach-provided verification helpers and raw logs remain outside this repository in local training records. No self-contained automated test suite is shipped here. These results do not replace independent simulator ground truth or course cloud acceptance.

## License

No reuse license has been selected. Package metadata is marked `UNLICENSED`; publishing the repository does not grant an open-source license.
