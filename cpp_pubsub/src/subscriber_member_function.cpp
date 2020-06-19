
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tutorial_interfaces/msg/num.hpp" 

using std::placeholders::_1;

/*
 *The subscriber node’s code is nearly identical to the publisher’s. 
 *Now the node is named minimal_subscriber, and the constructor uses the node’s create_subscription class to execute the callback.
 */
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_str_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&MinimalSubscriber::StringCallBack, this, _1));
    subscription_num_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
      "chatter", 10, std::bind(&MinimalSubscriber::NumCallBack, this, _1));
  }

private:

  /*
   *the ChatterCallBack function receives the string message data published over the topic chatter,
   *and simply writes it to the console using the RCLCPP_INFO macro.
   */
  void StringCallBack(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void NumCallBack(const tutorial_interfaces::msg::Num::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Count: '%d'", msg->num);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_str_;
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_num_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
