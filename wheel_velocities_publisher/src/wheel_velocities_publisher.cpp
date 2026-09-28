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
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
      [this]() { publish_forward(); });
    RCLCPP_INFO(get_logger(), "CP16-C003 scaffold: complete publish_forward() before checking messages");
  }

private:
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
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}