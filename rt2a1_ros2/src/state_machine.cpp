#include <functional>
#include <memory>
#include <cinttypes>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "rt2a1_ros2/srv/command.hpp"
#include "rt2a1_ros2/srv/position.hpp"
#include "rt2a1_ros2/srv/random_position.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std::chrono_literals;


namespace rt2a1_ros2{

class MinimalStateMachine : public rclcpp::Node
{
public:
  MinimalStateMachine(const rclcpp::NodeOptions & options): Node("minimal_state_machine", options)
  {
  	  reached_goal = true;
  	  start = false;	
  	  
      RCLCPP_INFO(this->get_logger(), "In class constructor"); 
      service_ = this->create_service<rt2a1_ros2::srv::Command>(
      "/user_interface", std::bind(&MinimalStateMachine::user_interface, this, _1, _2));
      
      client_rp = this->create_client<rt2a1_ros2::srv::RandomPosition>("/position_server");
      while (!client_rp->wait_for_service(std::chrono::seconds(1)))
      {
          if (!rclcpp::ok()) 
    	  {
      	      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
              return;
    	  }
      	  RCLCPP_INFO(this->get_logger(), "waiting for /position_server service to appear...");
      }
      
      client_p = this->create_client<rt2a1_ros2::srv::Position>("/go_to_point");
      while (!client_p->wait_for_service(std::chrono::seconds(1)))
      {
          if (!rclcpp::ok()) 
    	  {
      	      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
              return;
    	  }
      	  RCLCPP_INFO(this->get_logger(), "waiting for /go_to_point service to appear...");
      }
      
      this->request_rp = std::make_shared<rt2a1_ros2::srv::RandomPosition::Request>();
      this->request_p = std::make_shared<rt2a1_ros2::srv::Position::Request>();
     
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalStateMachine::send_goal, this));
  }
   
private:
  
  void send_goal()
  {
  	if(reached_goal and start)
  	{
		  this->request_rp->x_max = 5.0;
		  this->request_rp->x_min = -5.0;
		  this->request_rp->y_max = 5.0;
		  this->request_rp->y_min = -5.0;
		             	
		  reached_goal = false;
			         
		  using ServiceResponseFutureClient_rp = rclcpp::Client<rt2a1_ros2::srv::RandomPosition>::SharedFuture;
		  auto client_rp_callback = [this](ServiceResponseFutureClient_rp future2) 
		  {
				this->request_p->x = future2.get()->x;
				this->request_p->y = future2.get()->y;
				this->request_p->theta = future2.get()->theta;
				std::cout << "\nGoing to the position: x= " << this->request_p->x << " y= " <<this->request_p->y << " theta = " <<this->request_p->theta << std::endl;
		  };
		  auto response_rp = client_rp->async_send_request(this->request_rp, client_rp_callback); 
		  
		  using ServiceResponseFutureClient_p = rclcpp::Client<rt2a1_ros2::srv::Position>::SharedFuture;
		  auto client_p_callback = [this](ServiceResponseFutureClient_p future1) 
		  {
			   (void)future1;
			   reached_goal = true;
			   RCLCPP_INFO(this->get_logger(), "Position reached.");
		  };
		  auto response_p = client_p->async_send_request(this->request_p, client_p_callback);
	}
	else
	{
		return;
	}
  }
  
  void user_interface(
  const std::shared_ptr<rt2a1_ros2::srv::Command::Request> request,
  const std::shared_ptr<rt2a1_ros2::srv::Command::Response> response)
  {
      if (request->command == "start")
      {
          start = true;         
      }
      else 
      {
          start = false;
      }
      response->ok = start;
      std::cout << start << std::endl;
  }
  
  bool start;
  bool reached_goal;
  rclcpp::Service<rt2a1_ros2::srv::Command>::SharedPtr service_;
  rclcpp::Client<rt2a1_ros2::srv::RandomPosition>::SharedPtr client_rp;
  rclcpp::Client<rt2a1_ros2::srv::Position>::SharedPtr client_p;
  std::shared_ptr<rt2a1_ros2::srv::RandomPosition::Request> request_rp;
  std::shared_ptr<rt2a1_ros2::srv::Position::Request> request_p;
  rclcpp::TimerBase::SharedPtr timer_;
  
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2a1_ros2::MinimalStateMachine) 

