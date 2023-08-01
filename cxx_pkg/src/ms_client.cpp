// Copyright 2021 Seoul Business Agency Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdlib>
#include <memory>

#include "study_msgs/srv/move_shapes.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using MoveControl = study_msgs::srv::MoveShapes;

class MSClient : public rclcpp::Node
{
 public:
  MSClient(/* args */);
  ~MSClient();
  auto get_result_future(std::string shape_in)
  {
    RCLCPP_WARN(get_logger(), "Input Info");
    RCLCPP_INFO(get_logger(), "move_shape : %s\n", shape_in.c_str());

    m_request->shape.data = shape_in;

    return m_client->async_send_request(m_request);
  }

 private:
  rclcpp::Client<MoveControl>::SharedPtr m_client;
  std::shared_ptr<MoveControl::Request> m_request;
};

// # Request
// std_msgs/String shape
// ---
// # Response
// bool success

MSClient::MSClient(/* args */) : Node("move_shapes_client")
{
  m_client = create_client<MoveControl>("move_robot");
  m_request = std::make_shared<MoveControl::Request>();

  while (!m_client->wait_for_service(1s))
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");

  RCLCPP_INFO(get_logger(), "service available, waiting serice call");
}

MSClient::~MSClient()
{
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "usage: robot_turning_client [shape]");
    return 1;
  }

  auto basic_service_client = std::make_shared<MSClient>();
  auto result = basic_service_client->get_result_future(argv[1]);

  // Wait for the result.

  // rclcpp::executor::FutureReturnCode::SUCCESS -----> ROS2 foxy
  // rclcpp::FutureReturnCode::SUCCESS           -----> ROS2 humble

  if (rclcpp::spin_until_future_complete(basic_service_client, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result : %s",
                result.get()->success ? "True" : "False");
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service");

  rclcpp::shutdown();
  return 0;
}