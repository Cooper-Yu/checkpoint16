#include <array>
#include <algorithm>
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

// Coach支持：世界速度输出类型和教学参数；C043核心由学习者实现。
struct WorldVelocity { double vx; double vy; };
std::optional<WorldVelocity> compute_world_velocity(const PositionError & error)
{
  const double position_tolerance = 0.02;  // m
  const double max_speed = 0.1;           // m/s
  const double distance_gain = 0.5;       // 1/s
  // TODO CP16-C043：由error计算世界速度，先处理位置容差，再按比例限速。
  // error.ex / error.ey单位m；返回WorldVelocity{vx, vy}，单位m/s。
  // 支持API：std::hypot(a,b)求sqrt(a*a+b*b)，std::min(a,b)取较小值。
  const double rho = std::hypot(error.ex, error.ey);
  const double v = std::min(distance_gain * rho, max_speed);
  return rho <= position_tolerance ? WorldVelocity {0, 0} : WorldVelocity {v * error.ex / rho, v * error.ey / rho};
}

// Coach支持：单独命名输出坐标系，避免混用世界与车身速度。
struct BodyLinearVelocity { double vx; double vy; };
std::optional<BodyLinearVelocity> world_to_body(
  const WorldVelocity & world, double current_yaw)
{
  // TODO CP16-C044：使用当前朝向，将world转换为车身线速度。
  // current_yaw单位rad；std::cos()/std::sin()输入弧度。
  // 返回BodyLinearVelocity{vx, vy}；本函数不产生角速度。
  const double vx = world.vx * std::cos(current_yaw) + world.vy * std::sin(current_yaw);
  const double vy = -world.vx * std::sin(current_yaw) + world.vy * std::cos(current_yaw);

  return BodyLinearVelocity{vx, vy};
}

std::optional<double> compute_heading_error(double target_yaw, double current_yaw)
{
  const double pi = std::acos(-1.0);  // Coach支持：弧度制π。
  // TODO CP16-C045：计算目标减当前的朝向差，返回[-pi, pi]内的等价角差。
  // 可通过加减整圈(2*pi)处理越界，目标yaw可能已经累加超过一圈。
  // 此处只求角度误差，不计算角速度。恰好±pi时保留任一端点均可。
  return std::remainder(target_yaw - current_yaw, 2*pi);
}

std::optional<double> compute_angular_velocity(double heading_error)
{
  const double heading_tolerance = 0.05;  // rad
  const double heading_gain = 1.0;        // 1/s
  const double max_angular_speed = 0.5;   // rad/s
  // TODO CP16-C046：朝向容差内返回0，否则按比例并双向限幅，保留符号。
  // 支持API：std::abs(x)求绝对值；std::clamp(value, lower, upper)限制范围。
  const double angular_speed = heading_gain * heading_error;
  return std::abs(heading_error) <= heading_tolerance ? 0 : std::clamp(angular_speed, -max_angular_speed, max_angular_speed);
}

// Coach支持：WheelSpeeds按FL、FR、RL、RR排列，单位rad/s。
using WheelSpeeds = std::array<double, 4>;
std::optional<WheelSpeeds> body_to_wheels(
  const BodyLinearVelocity & body, double omega)
{
  const double wheel_radius = 0.05;  // m
  const double k = 0.085 + 0.135;    // m，前后/左右半距之和
  // TODO CP16-C047：将body.vx、body.vy、omega转换为四轮角速度。
  // 返回写法：return WheelSpeeds{前左表达式, 前右表达式, 后左表达式, 后右表达式};
  // 使用已学麦轮关系，保留各项符号与轮半径；本片不发布话题。
  const double vx = body.vx;
  const double vy = body.vy;
  return WheelSpeeds {
    (vx - vy - k * omega) / wheel_radius,
    (vx + vy + k * omega) / wheel_radius,
    (vx + vy - k * omega) / wheel_radius,
    (vx - vy + k * omega) / wheel_radius,
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
