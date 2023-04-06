#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("number", 10);

    std_msgs::msg::Int32 msg;
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        msg.data++;
        RCLCPP_INFO(node->get_logger(), "I publish: %d", msg.data);
        publisher->publish(msg);
    }

    rclcpp::shutdown();
}