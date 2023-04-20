#include <thread>
#include <mutex>
#include <memory>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "semi_robot_interfaces/msg/robot_state.hpp"

using semi_robot_interfaces::msg::RobotState;
using geometry_msgs::msg::Twist;

rclcpp::Node::SharedPtr g_node;
std::mutex g_state_mutex;
RobotState::SharedPtr g_state_ptr;

void onRobotStateSubscribed(const RobotState::SharedPtr msg)
{
    g_state_mutex.lock();
    g_state_ptr = msg;
    g_state_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("robot");
    auto vel_publisher = g_node->create_publisher<Twist>("~/cmd_vel", 10);
    auto state_subscriber = g_node->create_subscription<RobotState>("~/state", 10, onRobotStateSubscribed);
    std::thread([]{rclcpp::spin(g_node);}).detach();

    Twist vel;
    std::srand(std::time(nullptr));
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        g_state_mutex.lock();
        auto state_ptr = g_state_ptr;
        g_state_mutex.unlock();
        vel.linear.x = std::rand() % 10;
        vel.linear.y = std::rand() % 10;
        vel.linear.z = std::rand() % 10;
        vel.angular.x = std::rand() % 10;
        vel.angular.y = std::rand() % 10;
        vel.angular.z = std::rand() % 10;

        if(state_ptr.get()){
            RCLCPP_INFO(g_node->get_logger(), "enc[%d, %d]", state_ptr->encoder[0], state_ptr->encoder[1]);
            RCLCPP_INFO(g_node->get_logger(), "rpm[%d, %d]", state_ptr->rpm[0], state_ptr->rpm[1]);
        }

        vel_publisher->publish(vel);
    }
    rclcpp::shutdown();
}