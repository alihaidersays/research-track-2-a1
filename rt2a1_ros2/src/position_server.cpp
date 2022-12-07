#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rt2a1_ros2/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp" 
/*This head has to be placed after #include "rclcpp/rclcpp.hpp" this so that firstly 
 *header can be build correctly. */

#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/* This example  creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

namespace rt2a1_ros2{

class MinimalPositionServer : public rclcpp::Node
{
public:
  MinimalPositionServer(const rclcpp::NodeOptions & options) 
  : Node("minimal_position_server", options)
  {
    service_ = this->create_service<rt2a1_ros2::srv::RandomPosition>(
      "/position_server", std::bind(&MinimalPositionServer::myrandom, this, _1, _2));
  }

private:

  double randMToN(double M, double N)
  {  
      return M+(rand()/(RAND_MAX/(N-M))); 
  }

  bool myrandom(
  //const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2a1_ros2::srv::RandomPosition::Request> request,
  const std::shared_ptr<rt2a1_ros2::srv::RandomPosition::Response> response)
  {
      //(void)request_header;
      response->x = randMToN(request->x_min, request->x_max);
      response->y = randMToN(request->y_min, request->y_max);
      response->theta = randMToN(-3.14, 3.14);
      return true;       
  }

  rclcpp::Service<rt2a1_ros2::srv::RandomPosition>::SharedPtr service_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2a1_ros2::MinimalPositionServer)

