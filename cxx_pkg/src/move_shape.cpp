#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "study_msgs/srv/move_shapes.hpp"

using Twist = geometry_msgs::msg::Twist;
using MoveShapes = study_msgs::srv::MoveShapes;

class MoveRobot : public rclcpp::Node
{
 public:
  MoveRobot(const std::string& model);
  ~MoveRobot();
  void responseCallBack(std::shared_ptr<MoveShapes::Request> request, std::shared_ptr<MoveShapes::Response> response)
  {
    rclcpp::WallRate rate(0.4);  // 0.4Hz
    if (request->shape.data == "go") // shape는 std_msgs/String 형태로 정의되어 있음 -> data 를 사용해야 함
    {
      moveRobot(0.5, 0.0);
      rate.sleep();
      stopRobot();
      RCLCPP_INFO(get_logger(), "Robot is moving in go shape");
      response->success = true;
    }
    else if (request->shape.data == "turnleft")
    {
      turnRobot(1.0);
      rate.sleep();
      stopRobot();
      RCLCPP_INFO(get_logger(), "Robot is moving in turnleft shape");
      response->success = true;
    }
    else if (request->shape.data == "turnright")
    {
      turnRobot(-1.0);
      rate.sleep();
      stopRobot();
      RCLCPP_INFO(get_logger(), "Robot is moving in turnright shape");
      response->success = true;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "The shape is not defined");
      response->success = false;
    }
  }
  void moveRobot(float linear_x, float angular_z)
  {
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;

    cmd_pub->publish(twist_msg);
  }
  void stopRobot()
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_pub->publish(twist_msg);
  }
  void turnRobot(float angular_z)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = angular_z;
    cmd_pub->publish(twist_msg);
  }

 private:
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub;
  Twist twist_msg;
  rclcpp::Service<MoveShapes>::SharedPtr move_shapes_service;
};

// # Request
// std_msgs/String shape
// ---
// # Response
// bool success

MoveRobot::MoveRobot(const std::string& model) : Node("move_shapes_server")
{
  RCLCPP_INFO(get_logger(), "Move Shapes Server Node Start!");
  cmd_pub = create_publisher<Twist>(model + "/cmd_vel",10);
  move_shapes_service = create_service<MoveShapes>(
        "move_robot", std::bind(&MoveRobot::responseCallBack, this,
                                std::placeholders::_1, std::placeholders::_2));
}

MoveRobot::~MoveRobot()
{
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::string& model = std::getenv("ROBOT_MODEL");
  rclcpp::spin(std::make_shared<MoveRobot>(model));
  rclcpp::shutdown();

  return 0;
}