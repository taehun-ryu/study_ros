#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "study_msgs/srv/image_capture.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
using ImageCapture = study_msgs::srv::ImageCapture;

class ParkingNode : public rclcpp::Node
{
 public:
  ParkingNode(const std::string& model);
  auto get_result_future(bool edge_in)
  {
    RCLCPP_WARN(get_logger(), "Image Capture Service Called!");
    m_request->edge = edge_in;

    return m_client->async_send_request(m_request);
  }
  // subscribe 시마다 실행될 callback입니다.
  void subCallBack(const LaserScan::SharedPtr msg)
  {
    auto forward_distance = (msg->ranges)[360];

		if (forward_distance > 0.8)
    {
      moveRobot(forward_distance);
    }
    else
    {
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
  rclcpp::Client<ImageCapture>::SharedPtr m_client;
  std::shared_ptr<ImageCapture::Request> m_request;
};

ParkingNode::ParkingNode(const std::string& model) : Node("robot_parking_node")
{
  RCLCPP_INFO(get_logger(), "Parking Node Created about ");
  cmd_pub = create_publisher<Twist>(model + "/cmd_vel",10);
  laser_sub = create_subscription<LaserScan>(model + "/scan",10,
                std::bind(&ParkingNode::subCallBack, this, std::placeholders::_1));
  m_client = create_client<ImageCapture>("capture_image");
  m_request = std::make_shared<ImageCapture::Request>();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::string& model = std::getenv("ROBOT_MODEL");
  rclcpp::spin(std::make_shared<ParkingNode>(model));

  rclcpp::init(argc, argv);
  auto parking_node = std::make_shared<ParkingNode>(model);
  auto result = parking_node->get_result_future(true);
  if (rclcpp::spin_until_future_complete(parking_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result : %s",
            result.get()->success ? "True" : "False");
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
              "Failed to call service");
  rclcpp::shutdown();

  return 0;
}