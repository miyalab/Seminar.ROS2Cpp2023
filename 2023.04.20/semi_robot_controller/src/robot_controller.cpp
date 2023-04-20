#include <thread>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "semi_robot_interfaces/msg/robot_state.hpp"

using semi_robot_interfaces::msg::RobotState;
using geometry_msgs::msg::Twist;

std::mutex g_state_mutex;
RobotState::SharedPtr g_state_ptr = std::make_shared<RobotState>();

void onRobotStateSubscribed(const RobotState::SharedPtr msg)
{
    g_state_mutex.lock();
    g_state_ptr = msg;
    g_state_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_controller");
    auto vel_publisher = node->create_publisher<Twist>("~/state", 10);
    auto state_subscriber = node->create_subscription<RobotState>("~/cmd_vel", 10, onRobotStateSubscribed);

    std::srand(std::time(nullptr));
    Twist vel;
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

        vel_publisher->publish(vel);
    }
}