#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"  
#include <memory>
/**
 * The add function adds two integers from the request 
 * and gives the sum to the response, while notifying the console of its status using logs.
 **/

void AddThreeInts(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
          std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                       // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",   // CHANGE
                request->a, request->b, request->c);                                          // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

/**
 * initialize ROS 2 C++ client library
 * Create a node named 'add_two_ints_server'
 * Creates a service named add_two_ints for that node 
 * and automatically advertises it over the networks with the &Add method:
 * */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server"); 
  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service_add_three =                 // CHANGE
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &AddThreeInts); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}