#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Coach支持：节点接线、输入检查、轮序别名和消息发布。
// 学习者逐分量实现计算；当前不是完整运动学模型，不用于仿真联动。
class KinematicModel : public rclcpp::Node
{
public:
  KinematicModel() : Node("kinematic_model")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wheel_speed", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        on_wheel_speed(*msg);
      });
  }

private:
  void on_wheel_speed(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() != 4 ||
        !std::all_of(msg.data.begin(), msg.data.end(),
          [](float value) { return std::isfinite(value); })) {
      RCLCPP_WARN(get_logger(), "Expected four finite wheel speeds in FL/FR/RL/RR order");
      return;
    }
    const double wheel_radius_m = 0.05;
    const double half_wheelbase_m = 0.085;
    const double half_track_m = 0.135;
    const double w_fl = msg.data[0];
    const double w_fr = msg.data[1];
    const double w_rl = msg.data[2];
    const double w_rr = msg.data[3];
    geometry_msgs::msg::Twist cmd;

    // TODO CP16-C035：使用上述变量计算vx，赋给cmd.linear.x。
    // C035前向分量已验证；保持学习者公式。
    cmd.linear.x = wheel_radius_m / 4 * (w_fl + w_fr + w_rl + w_rr);

    // TODO CP16-C036：计算vy，赋给cmd.linear.y；angular.z暂留零。
    cmd.linear.y = wheel_radius_m / 4 * (-w_fl + w_fr + w_rl - w_rr);

    // TODO CP16-C037：计算车身角速度，赋给cmd.angular.z。
    // k是两个半距之和，使用给定变量，不写死结果。
    cmd.angular.z = wheel_radius_m / (4 * (half_wheelbase_m + half_track_m)) * (-w_fl + w_fr - w_rl + w_rr);
    publisher_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}
