#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp" 
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_ints_client X Y or X Y Z");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client"); 
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                        // CHANGE
    node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");  
/**
 * Next, the request is created. 
 * Its structure is defined by the .srv file mentioned earlier.
 * */
  
  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();               // CHANGE
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);    
/**
 * The while loop gives the client 1 second to search for service nodes in the network. 
 * If it can’t find any, it will continue waiting.
 **/
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  /**
   *  The client send its request
   *  The node spins until it receives response or fails
   **/
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
  }

  rclcpp::shutdown();
  return 0;
}