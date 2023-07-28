#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistPub : public rclcpp::Node
{
 public:
  TwistPub(const std::string& model);
  void move_robot()
  {
    m_twist_msg.linear.x = 0.5;
    m_twist_msg.angular.z = 1.0;
    m_pub->publish(m_twist_msg);
  }
  void stop_robot()
  {
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_pub->publish(m_twist_msg);
    RCLCPP_INFO(get_logger(), "==== Stop Robot ====");
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
  rclcpp::TimerBase::SharedPtr m_timer;
  geometry_msgs::msg::Twist m_twist_msg;
  void timer_callback()
  {
    move_robot();
  }
};

TwistPub::TwistPub(const std::string& model): Node("cmd_vel_pub_node")
{
  RCLCPP_INFO(get_logger(), "Cmd_vel Pub Node Created");
  m_pub = create_publisher<geometry_msgs::msg::Twist>(model + "/cmd_vel", 10);
  m_timer = create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&TwistPub::timer_callback, this));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::string& model = std::getenv("ROBOT_MODEL");
  auto twist_pub = std::make_shared<TwistPub>(model);
  auto t_start = twist_pub->now();
  auto t_now = twist_pub->now();
  auto stop_time = 5.0;
  while ((t_now - t_start).seconds() < stop_time) {
    t_now = twist_pub->now();
    twist_pub->move_robot();
    RCLCPP_INFO(twist_pub->get_logger(), "%f Seconds Passed", (t_now - t_start).seconds());
  }
  twist_pub->stop_robot();
  rclcpp::shutdown();

  return 0;
}