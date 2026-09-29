#include <cmath>
#include <memory>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Coach支持：数据结构、订阅与姿态提取；当前切片不发布运动命令。
struct Pose2D { double x; double y; double yaw; };
struct SegmentDelta { double dphi; double dx; double dy; };

std::optional<Pose2D> make_segment_target(
  const Pose2D & start, const SegmentDelta & delta)
{
  // TODO CP16-C041：用start与delta构造并返回目标Pose2D。
  // Pose2D字段顺序：x、y、yaw；SegmentDelta字段顺序：dphi、dx、dy。
  // 返回语法：return Pose2D{表达式1, 表达式2, 表达式3};
  // 本片只累加目标朝向；最短角度误差在后续控制切片处理。

  return Pose2D {
    start.x + delta.dx,
    start.y + delta.dy,
    start.yaw + delta.dphi
  };
}

// Coach支持：C042只处理世界坐标位置误差，不处理朝向。
struct PositionError { double ex; double ey; };
std::optional<PositionError> compute_position_error(
  const Pose2D & target, const Pose2D & current)
{
  // TODO CP16-C042：按ex、ey顺序返回目标相对当前位置的误差。
  // 返回写法：return PositionError{表达式1, 表达式2};


  return PositionError {
    target.x - current.x,
    target.y - current.y
  };
}

class EightTrajectory : public rclcpp::Node
{
public:
  EightTrajectory() : Node("eight_trajectory")
  {
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (target_) return;  // Coach支持：本片仅在初始化时保存一次目标。
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(norm2) || norm2 < 1e-12) return;
        const double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                      norm2 - 2.0*(q.y*q.y + q.z*q.z));
        const Pose2D start{p.x, p.y, yaw};
        const SegmentDelta first_segment{0.0, 1.0, -1.0};
        target_ = make_segment_target(start, first_segment);
        if (!target_) {
          RCLCPP_WARN_ONCE(get_logger(), "C041 target calculation is not implemented yet");
          return;
        }
        RCLCPP_INFO(get_logger(), "Target: x=%.3f y=%.3f yaw=%.3f",
                    target_->x, target_->y, target_->yaw);
      });
  }
private:
  std::optional<Pose2D> target_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}
