#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("parameter");

    // パラメータ読み込み
    auto val = node->declare_parameter<int>("int_val", 0);
    auto str = node->declare_parameter<std::string>("string", "hello");
    auto num = node->declare_parameter<double>("double_val", 0.0);

    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        RCLCPP_INFO(node->get_logger(), "val: %ld", val);
        RCLCPP_INFO(node->get_logger(), "str: %s", str.c_str());
        RCLCPP_INFO(node->get_logger(), "num: %lf", num);
    }
    rclcpp::shutdown();
}