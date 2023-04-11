#include <rclcpp/rclcpp.hpp>
#include <tutorial_interfaces/msg/int32_stamped.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<tutorial_interfaces::msg::Int32Stamped>("number", 10);

    tutorial_interfaces::msg::Int32Stamped msg;
    msg.header.frame_id = "int32 type data";
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        msg.header.stamp = node->now();
        msg.data++;
        RCLCPP_INFO(node->get_logger(), "I publish: %d", msg.data);
        publisher->publish(msg);
    }

    rclcpp::shutdown();
}