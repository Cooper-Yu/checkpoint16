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
    RCLCPP_INFO(get_logger(), "CP16-C017 scaffold: complete update_motion() before checking timing");
  }

private:
  void update_motion(double elapsed_seconds)
  {
    // TODO CP16-C017：根据本段已过秒数，选择调用已有的运动发布函数。
    // 要求：本段开始后的前3秒持续前进，达到3秒后持续发布停止。
    // 只写时间判断与函数调用；不要sleep，不重新计算四轮速度。
    if (elapsed_seconds < 3) {
      publish_forward();
    } else {
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
  std::chrono::steady_clock::time_point segment_started_at_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}