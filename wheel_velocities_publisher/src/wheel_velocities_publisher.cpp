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
      [this]() { publish_stop(); });
    RCLCPP_INFO(get_logger(), "Publishing four-wheel stop commands");
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