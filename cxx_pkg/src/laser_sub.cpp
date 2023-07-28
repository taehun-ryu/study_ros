#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

class LaserSub : public rclcpp::Node
{
 public:
  LaserSub(const std::string& model);

  void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::cout << (msg->ranges).size() << std::endl;
    std::cout << '[';
    for (auto e : msg->ranges)
    {
      std::cout << e << ',';
    }
    std::cout << ']' << std::endl;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;
};

LaserSub::LaserSub(const std::string& model): Node("topic_sub_oop_node")
{
  m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      model + "/scan", 10,
      std::bind(&LaserSub::sub_callback, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::string& model = std::getenv("ROBOT_MODEL");
  rclcpp::spin(std::make_shared<LaserSub>(model));
  rclcpp::shutdown();

  return 0;
}