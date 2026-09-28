# Checkpoint 16 — Mobile Robot Kinematics

ROSBot XL麦克纳姆轮学习项目。目标环境：Ubuntu 22.04 / ROS 2 Humble、Gazebo Fortress。核心代码由学习者增量实现，Coach提供脚手架、评审及验证。仿真依赖单独管理，不包含课程原始材料。

## 当前状态

- Task1轮速发布、正向运动学和双节点launch已实现，本地消息及仿真里程计验证通过。
- Task2 eight_trajectory尚未实现。
- 独立Gazebo模型位姿及课程云端最终验收待完成；不代表整个Checkpoint通过。

## 数据流与约定

wheel_velocities_publisher → /wheel_speed → kinematic_model → /cmd_vel → 仿真控制器

- /wheel_speed：std_msgs/msg/Float32MultiArray；顺序[FL, FR, RL, RR]（前左、前右、后左、后右），单位rad/s。
- /cmd_vel：geometry_msgs/msg/Twist；车身x向前、y向左，绕z轴逆时针为正。
- 发布器每100 ms检查阶段，依次前进、后退、左移、右移、顺时针、逆时针，每阶段约3秒，随后持续发布零速。检查周期与调度影响切换时刻。
- 平移幅值0.1 m/s，旋转幅值0.5 rad/s；这是按时间切换的序列，不使用里程计闭环调整。
- 模型拒绝长度非4或包含非有限数的输入。未实现输入超时停车；拒绝输入不会主动发布零速。

## 几何与模型

r=0.05 m；前后轮中心距0.17 m、左右轮中心距0.27 m；半距lx=0.085 m、ly=0.135 m，k=lx+ly=0.22 m。

    vx = r/4     * ( w_FL + w_FR + w_RL + w_RR)
    vy = r/4     * (-w_FL + w_FR + w_RL - w_RR)
    Ω  = r/(4*k) * (-w_FL + w_FR - w_RL + w_RR)

本地与课程云端三项运行几何参数已核对。尺寸图左右轮中心距269.69 mm与运行值270 mm略有不同，代码采用运行值。

## 构建与运行

在已配置ROS 2 Humble及colcon的Ubuntu 22.04中，将仓库放在~/ros2_ws/src/checkpoint16：

    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    colcon build --packages-select wheel_velocities_publisher kinematic_model
    source install/setup.bash
    ros2 launch kinematic_model kinematic_model.launch.py

launch只启动两个作业节点。可在另一个加载同一环境的终端检查：

    ros2 topic echo /wheel_speed
    ros2 topic echo /cmd_vel

接入Gazebo前，需要兼容的ROSBot XL麦克纳姆轮仿真，控制器订阅/cmd_vel；作业与仿真进程的ROS_DOMAIN_ID和ROS_LOCALHOST_ONLY必须一致。避免同时运行其他速度发布者。

本地验证使用独立checkpoint16_sim_ws、ROS_DOMAIN_ID=116、ROS_LOCALHOST_ONLY=1；仿真依赖和适配脚本不随仓库分发。反馈来自/rosbot_xl_base_controller/odom，并检查/odometry/filtered有输出。课程其他版本可能需要不同配置。

## 已执行验证（2026-09-29）

| 检查 | 结果 |
| --- | --- |
| 两个C++包构建 | 通过 |
| 模型9组输入：六向、两组混合、零速 | Twist各分量匹配 |
| 新输入[1,3,3,1]独立预测及运行 | (0.1 m/s, 0.05 m/s, 0 rad/s) |
| 双节点launch，隔离域117 | 213条轮速及213条Twist，六向及停止顺序正确 |
| Gazebo联调，域116 | 里程计平移约±0.1 m/s，转向约±0.5 rad/s，最终约零；7/7阶段通过 |

验证使用仓库外Coach辅助脚本，详细训练记录和原始日志保留在本地。目前仓库没有可直接复跑的自动化测试套件。控制器里程计符合预期不等同于独立模型位姿验证。

## 后续工作

- 核对独立仿真位姿及课程云端接口和运行。
- 按学习步骤实现eight_trajectory、位姿反馈与轨迹验收。
