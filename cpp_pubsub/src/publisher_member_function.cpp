#include <chrono>
#include <memory>

/* rclcpp/rclcpp.hpp include which allows to use the most common pieces of the ROS 2 system
 * std_msgs/msg/string.hpp, which includes the built-in message type used to publish data
 * */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using namespace std::chrono_literals;

/* This Class creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
/* The constructor initialize the node name as minimal_publisher, 
 * count_ is initialized to be 0
 * the publisher is initialized with the String message type, the topic name chatter
 * timer_ is initialized, which causes the TimerCallBack function to be executed twice a second
 * */
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_string_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    publisher_num_ = this->create_publisher<tutorial_interfaces::msg::Num>("chatter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::TimerCallBack, this));
  }

private:
  /*
   * The TimerCallBack function is where the message data is set and the messages are actually published. 
   * The RCLCPP_INFO macro ensures every published message is printed to the console.
   * */
  void TimerCallBack()
  {
    auto message_str = std_msgs::msg::String();
    auto message_num = tutorial_interfaces::msg::Num();
    message_num.num = this->count_++;
    message_str.data = "Hello, world! ";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' '%d'", message_str.data.c_str(), message_num.num );
    publisher_string_->publish(message_str);
    publisher_num_->publish(message_num);
    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_num_; 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
