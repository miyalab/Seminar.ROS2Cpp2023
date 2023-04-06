#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);								// プロセスの初期化
	auto node = rclcpp::Node::make_shared("helloworld");	// nodeの初期化
    
    // rclcpp::WallRate(１秒間あたりのループ回数; rclcpp::ok(); WallRate.sleep())
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
		RCLCPP_INFO(node->get_logger(), "Hello World!");	// ROS2版printf
	}
	rclcpp::shutdown();
}