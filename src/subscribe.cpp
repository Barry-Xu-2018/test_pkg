#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "rmw/rmw.h"

namespace test_pkg
{

using std::placeholders::_1;

class SubscribeComposableNode : public rclcpp::Node
{
public:
  explicit SubscribeComposableNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("subscribe_node", options)
  {
    //rmw_set_log_severity(RMW_LOG_SEVERITY_WARN);

    sub_ = this->create_subscription<std_msgs::msg::String>(
        "~/input", rclcpp::QoS(10), std::bind(&SubscribeComposableNode::topicHandler, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "End of constructor.");
  }

  ~SubscribeComposableNode() = default;

private:
  void topicHandler(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    static __thread bool output = false;
    if (output) {
      RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(gettid()) + ": " + msg->data);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace test_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(test_pkg::SubscribeComposableNode);
