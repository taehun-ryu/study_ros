#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "study_msgs/srv/move_shapes.hpp"

class MoveShapes : public rclcpp::Node
{
 public:
  MoveShapes(/* args */);
  ~MoveShapes();

 private:
  /* data */
};

MoveShapes::MoveShapes(/* args */) : Node("move_shapes_node")
{
}

MoveShapes::~MoveShapes()
{
}

int main()
{
  

  return 0;
}