#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class ParkingNode : public rclcpp::Node
{
 public:
  ParkingNode(const std::string& model);
  // subscribe 시마다 실행될 callback입니다.
  void subCallBack(const LaserScan::SharedPtr msg)
  {
    auto forward_distance = (msg->ranges)[360];

		if (forward_distance > 0.8) {
      moveRobot(forward_distance);
    } else {
      stopRobot();
      rclcpp::shutdown();
    }
  }
  void moveRobot(const float &forward_distance)
  {
    twist_msg.linear.x = 0.5;
    twist_msg.angular.z = 0.0;
    cmd_pub->publish(twist_msg);

    std::cout << "Distance from Obstacle ahead : " << forward_distance << std::endl;
  }
  void stopRobot()
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_pub->publish(twist_msg);

    RCLCPP_WARN(get_logger(), "Stop Robot and make Node FREE!");
  }

 private:
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_sub;
  Twist twist_msg;
};

ParkingNode::ParkingNode(const std::string& model) : Node("robot_parking_node")
{
  RCLCPP_INFO(get_logger(), "Parking Node Created about ");
  cmd_pub = create_publisher<Twist>(model + "/cmd_vel",10);
  laser_sub = create_subscription<LaserScan>(model + "/scan",10,
                std::bind(&ParkingNode::subCallBack, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::string& model = std::getenv("ROBOT_MODEL");
  rclcpp::spin(std::make_shared<ParkingNode>(model));
  rclcpp::shutdown();

  return 0;
}