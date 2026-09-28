#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// Coach辅助结构：节点、publisher、定时器、main；不包含学习者发布逻辑。
class WheelVelocitiesPublisher : public rclcpp::Node
{
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
    segment_started_at_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() {
        const double elapsed_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - segment_started_at_).count();
        update_motion(elapsed_seconds);
      });
    RCLCPP_INFO(get_logger(), "CP16-C020 scaffold: complete begin_backward() before checking timing");
  }

private:
  enum class MotionMode {
    Forward,
    Backward,
    Left,
    Right,
    Clockwise,
    Counterclockwise,
    Stop
  };

  MotionMode next_mode(MotionMode current)
  {
    // TODO CP16-C024：只返回下一模式，不更新成员变量或发布消息。
    // 顺序：Forward -> Backward -> Left -> Right -> Clockwise -> Counterclockwise -> Stop。
    // Stop之后仍为Stop；七种输入都应返回结果，可使用if / else if。
    switch (current) {
      case MotionMode::Forward:
        return MotionMode::Backward;
      case MotionMode::Backward:
        return MotionMode::Left;
      case MotionMode::Left:
        return MotionMode::Right;
      case MotionMode::Right:
        return MotionMode::Clockwise;
      case MotionMode::Clockwise:
        return MotionMode::Counterclockwise;
      case MotionMode::Counterclockwise:
        return MotionMode::Stop;
      case MotionMode::Stop:
        return MotionMode::Stop;
    }
    return MotionMode::Stop;  // Coach支持：异常枚举值的返回兜底。
  }

  void begin_backward()
  {
    // TODO CP16-C020：进入后退时，更新mode_和segment_started_at_。
    // mode_使用MotionMode枚举；起点使用当前steady_clock时间。
    // 只完成这两个状态更新，消息发布由下面的辅助调度处理。
    mode_ = MotionMode::Backward;
    segment_started_at_ = std::chrono::steady_clock::now();

  }

  void update_motion(double elapsed_seconds)
  {
    // Coach调度支持：沿用C017已通过的3秒判断，扩展到两段。
    // C020学习者负责begin_backward()中的切换状态更新。
    if (mode_ == MotionMode::Forward) {
      if (elapsed_seconds < 3.0) {
        publish_forward();
      } else {
        begin_backward();
        publish_backward();
      }
      return;  // 本次elapsed属于前进段，不再拿它判断后退段。
    }
    if (elapsed_seconds < 3.0) {
      publish_backward();
    } else {
      // TODO CP16-C023：在此更新mode_，使状态记录与停止阶段一致。
      mode_ = MotionMode::Stop;
      publish_stop();
    }
  }

  void publish_stop()
  {
    // CP16-C001：停止消息已通过验证，保留学习者代码。
    // data顺序约定：[前左、前右、后左、后右]；单位rad/s。
    // 仅完成此处。不要修改辅助结构，也不要加入六段运动。
    std_msgs::msg::Float32MultiArray stop_msg;
    stop_msg.data = {0, 0, 0, 0};
    publisher_->publish(stop_msg);


  }

  void publish_forward()
  {
    const float forward_speed_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C003：由上述量计算轮速，构造并发布纯前进的四轮消息。
    // 数组：[前左、前右、后左、后右]；正轮速约定驱动车身向前。
    std_msgs::msg::Float32MultiArray forward_msg;
    float wheel_speed_radps = forward_speed_mps / wheel_radius_m;
    forward_msg.data = {wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(forward_msg);
  }
  void publish_backward()
  {
    const float speed_magnitude_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C005：使用给定变量，计算并发布纯后退的四轮速度。
    // 沿用数组[前左、前右、后左、后右]与正轮速驱动车身向前的约定。
    std_msgs::msg::Float32MultiArray backward_msg;
    float wheel_speed_radps = -speed_magnitude_mps / wheel_radius_m;
    backward_msg.data = {wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(backward_msg);
  }
  void publish_left()
  {
    const float left_speed_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C007：使用给定变量，计算并发布纯左移的四轮速度。
    // 数组顺序：[前左、前右、后左、后右]，单位rad/s。
    std_msgs::msg::Float32MultiArray left_msg;
    float wheel_speed_radps = left_speed_mps / wheel_radius_m;
    left_msg.data = {-wheel_speed_radps, wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps};
    publisher_->publish(left_msg);
  }

  void publish_right()
  {
    const float right_speed_magnitude_mps = 0.10f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C008：使用给定变量，计算并发布纯右移的四轮速度。
    // 数组顺序：[前左、前右、后左、后右]；单位rad/s。
    std_msgs::msg::Float32MultiArray right_msg;
    float wheel_speed_radps = right_speed_magnitude_mps / wheel_radius_m;
    right_msg.data = {wheel_speed_radps, -wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(right_msg);
  }
  void publish_clockwise()
  {
    const float body_angular_velocity_radps = -0.50f;
    const float half_wheelbase_m = 0.085f;
    const float half_track_m = 0.135f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C011：由给定参数计算并发布顺时针原地转向的四轮速度。
    // 数组：[前左、前右、后左、后右]；单位rad/s，不写死轮速。
    std_msgs::msg::Float32MultiArray clockwise_msg;
    float wheel_speed_radps = (half_wheelbase_m + half_track_m) * body_angular_velocity_radps / wheel_radius_m;
    clockwise_msg.data = {-wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(clockwise_msg);
  }

  void publish_counterclockwise()
  {
    const float body_angular_velocity_radps = 0.50f;
    const float half_wheelbase_m = 0.085f;
    const float half_track_m = 0.135f;
    const float wheel_radius_m = 0.05f;
    // TODO CP16-C013：由给定参数计算并发布逆时针原地转向的四轮速度。
    // 数组：[前左、前右、后左、后右]；单位rad/s，不写死轮速。
    std_msgs::msg::Float32MultiArray counterclockwise_msg;
    float wheel_speed_radps = (half_wheelbase_m + half_track_m) * body_angular_velocity_radps / wheel_radius_m;
    counterclockwise_msg.data = {-wheel_speed_radps, wheel_speed_radps, -wheel_speed_radps, wheel_speed_radps};
    publisher_->publish(counterclockwise_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MotionMode mode_ = MotionMode::Forward;  // 本切片仅调度前进和后退，后退满3秒停止。
  std::chrono::steady_clock::time_point segment_started_at_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}