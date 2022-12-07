// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <memory>
#include <cstdlib>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;


namespace rt2a1_ros2{

class MiniClient : public rclcpp::Node
{
public:
  
  MiniClient(const rclcpp::NodeOptions & options)
  : Node("mini_client", options)
  {
    client_ = this->create_client<AddTwoInts>("add_two_ints");
    
    while (!client_->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
   timer_ = this->create_wall_timer(
      2000ms, std::bind(&MiniClient::call_server, this));
  }
  
private:

void call_server()
  {
  this->request_ = std::make_shared<AddTwoInts::Request>();
  this->request_->a = std::rand();
  this->request_->b = std::rand();
  
  
  //In order to wait for a response to arrive, we need to spin().
  // However, this function is already being called from within another spin().
  // Unfortunately, the current version of spin() is not recursive and so we
  // cannot call spin() from within another spin().
  // Therefore, we cannot wait for a response to the request we just made here
  // within this callback, because it was executed by some other spin function.
  // The workaround for this is to give the async_send_request() method another
  // argument which is a callback that gets executed once the future is ready.
  // We then return from this callback so that the existing spin() function can
  // continue and our callback will get called once the response is received.
   using ServiceResponseFuture =
    rclcpp::Client<AddTwoInts>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get()->sum);
    };
  auto future_result = client_->async_send_request(this->request_, response_received_callback);
  }
  rclcpp::Client<AddTwoInts>::SharedPtr client_; 
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<AddTwoInts::Request> request_;
};

}


RCLCPP_COMPONENTS_REGISTER_NODE(rt2a1_ros2::MiniClient) 

