# Checkpoint 16 — Mobile Robot Kinematics

ROSBot XL mecanum-wheel exercises for Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress. Core implementations were written incrementally by the learner with coaching, scaffolding, and verification support. Course materials and simulator dependencies are not included.

## Status

- Official evaluation on 2026-09-30: **8.5/10** (Task 1: 4.5/5; Task 2: 4/5). The score has not been updated; cloud verification and official re-evaluation of these repairs are pending.
- Task 1 settling intervals were added and checked locally; the learner also confirmed the cloud motion sequence and final visual stop.
- Task 2 now coordinates translation and rotation, limits changes in linear velocity, and shuts down its launch after the final stop interval. A local full eight-waypoint run and independent Gazebo pose comparison passed the diagnostic checks below.
- Launch uses `/odometry/filtered` by default. In the course cloud environment, raw controller odometry previously disagreed with the actual model pose; the underlying discrepancy remains undiagnosed.

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
- The Task 2 node uses `/odom` (`nav_msgs/msg/Odometry`) internally. The launch defaults to remapping it to `/odometry/filtered`, selected after the cloud model-pose comparison and learner runtime confirmation. The `odom_topic` argument can override this mapping.
- Odometry positions, goals, and world-frame velocities use the same fixed odometry frame. No TF conversion is performed.
- Run only one wheel-command source at a time. Do not run the Task 1 publisher alongside Task 2.

## Kinematics and control

The configured wheel radius is `r = 0.05 m`. Wheel-center distances are `0.17 m` front-to-rear and `0.27 m` left-to-right, giving `k = 0.085 + 0.135 = 0.22 m`. These parameters were checked against local and course cloud configurations. The drawing's lateral dimension is 269.69 mm; the code uses the configured 270 mm.

```text
vx = r/4     * ( w_FL + w_FR + w_RL + w_RR)
vy = r/4     * (-w_FL + w_FR + w_RL - w_RR)
omega = r/(4*k) * (-w_FL + w_FR - w_RL + w_RR)
```

Task 1 checks stages every 100 ms, commanding each motion for approximately 3 seconds of wall time. Each motion is followed by zero wheel commands: a 0.5-second settling interval before the next motion, and a 1-second interval after the final counterclockwise motion before entering Stop. Translation magnitude is 0.1 m/s and rotation magnitude is 0.5 rad/s. It continuously publishes zero in Stop.

Task 2 initializes its first goal from the first valid pose. Subsequent goals accumulate the configured world-frame increments from the previous planned goal, rather than from the actual stopping position. Translation and rotation can occur simultaneously.

| Setting | Value |
| --- | --- |
| Control period | 100 ms |
| Position tolerance | 0.02 m |
| Heading tolerance | 0.05 rad |
| Distance gain | 0.8 /s |
| Maximum linear speed | 0.4 m/s |
| Heading gain after position arrival | 1.0 /s |
| Maximum angular speed | 0.5 rad/s |
| Feedback timeout | 0.5 s, steady wall time |
| Linear velocity vector change | At most 0.02 m/s per control callback |
| Final zero-command interval | At least 1 s, steady wall time |

Desired linear speed is proportional to distance, capped at 0.4 m/s. Outside position tolerance, each callback limits the length of the world velocity change vector to 0.02 m/s. This is a per-callback command limit, not a fixed simulation-time acceleration bound when the real-time factor changes. The limited velocity is rotated into the body frame.

Outside position tolerance, angular velocity is `heading_error * speed / distance`, capped symmetrically at 0.5 rad/s. It uses the limited linear speed and shortest wrapped heading error. Inside position tolerance, translation stops and proportional heading correction finishes any remaining turn. Angular velocity is zero inside heading tolerance. A waypoint completes only when both tolerances are satisfied.

The controller sends zero during waypoint transitions and resets its velocity-change state on every stop. After the eighth waypoint, it continues publishing zero for at least one second, then exits. The launch listens for the trajectory process exit and shuts down the remaining converter node. The external simulator is unaffected. An unexpected trajectory process exit also triggers launch cleanup; cleanup alone is not evidence of successful completion.

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

Task 2 with filtered odometry (no argument needed):

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
4. Observe all eight waypoints, the two rotation segments, and the final stop followed by automatic launch exit. Confirm each position and heading against the course requirements.

Useful inspection commands, in a separately sourced terminal:

```bash
ros2 topic info /wheel_speed --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic echo /wheel_speed
ros2 topic echo /odometry/filtered
```

Use the selected feedback topic if you override the default. The learner reported that the cloud trajectory was normal with filtered feedback. Raw controller odometry previously converged internally while the Gazebo robot followed an incorrect path; do not use that convergence alone as acceptance evidence. The underlying raw-odometry discrepancy remains undiagnosed.

## Local validation (2026-09-30)

Environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, a single ROSBot XL in an empty world, filtered odometry feedback. These are local diagnostic results, not an official grading result or a guarantee for the cloud simulator.

| Check | Observed result |
| --- | --- |
| Build and helper checks | Coordinate conversion, wheel calculations, coordinated control, smoothing, and bounded ideal-loop checks passed |
| Feedback recovery | Stale feedback produces zero commands; restored feedback restarts the smoothed command at 0.02 m/s |
| Final completion with controlled ROS input | No early exit before the last position and heading; 11 final zero-wheel messages over approximately 1 s; zero commands delivered through the converter |
| Launch cleanup | Trajectory and converter finished cleanly; both command publishers disappeared |
| Full eight-waypoint Gazebo run | All eight filtered poses entered the 0.02 m / 0.05 rad tolerances in order; waypoint 8 reached at about 52.1 s wall time |
| Autonomous launch exit | About 53.4 s after launch; no harness stop command was needed for successful completion |
| Independent Gazebo position error at each arrival | Approximately 2.23, 1.91, 1.87, 2.44, 1.85, 1.11, 1.12, 1.57 cm |
| Independent endpoint after exit | Approximately 1.51 cm position error and 0.04368 rad (2.50 degrees) heading error |
| Rotation during w3 to w4 | Actual midpoint yaw approximately -0.870 rad, confirming rotation during translation |

Ground truth was aligned using the initial translation offset after checking that the initial yaw difference was below 0.001 rad. Arrival sample time differences were 3–7 ms. The diagnostics used a 4 cm actual-position bound, which is not a course-specified tolerance. Actual position errors can exceed the controller's 2 cm feedback tolerance.

With the same local setup, raising the distance gain from 0.5 to 0.8 while keeping the 0.4 m/s speed cap and smoothing reduced a four-waypoint trial from 38.3 s to 26.3 s; waypoint-4 actual error remained around 2.3–2.4 cm. This is a single-run comparison, not a statistical performance claim. The final complete run verified the remaining waypoints and automatic exit separately.

Earlier Task 1 six-direction, wheel-topic, and learner coordinate-transform checks remain recorded in the training notes. Coach-provided verification helpers and raw logs remain outside this repository; no self-contained automated test suite is shipped here. Cloud testing of this revision and official re-evaluation remain pending.

## License

No reuse license has been selected. Package metadata is marked `UNLICENSED`; publishing the repository does not grant an open-source license.
