#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <random>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto pose_publisher = node->create_publisher<geometry_msgs::msg::Pose>("~/pose", 10);
    auto point_publisher = node->create_publisher<geometry_msgs::msg::Point>("~/point", 10);

    std::srand(std::time(nullptr));
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Point point;
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        point.x = std::rand() % 10;
        point.y = std::rand() % 10;
        point.z = std::rand() % 10;
        pose.position.x = std::rand() % 10;
        pose.position.y = std::rand() % 10;
        pose.position.z = std::rand() % 10;
        pose_publisher->publish(pose);
        point_publisher->publish(point);
        RCLCPP_INFO(node->get_logger(), "I published point: [%lf, %lf, %lf]", point.x, point.y, point.z);
        RCLCPP_INFO(node->get_logger(), "I published pose : [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", 
            pose.position.x, pose.position.y, pose.position.z, 
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
        );
    }
}