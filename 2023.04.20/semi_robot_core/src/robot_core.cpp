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
std::mutex g_vel_mutex;
Twist::SharedPtr g_vel_ptr = std::make_shared<Twist>();

void onCmdVelSubscribed(const Twist::SharedPtr msg)
{
    g_vel_mutex.lock();
    g_vel_ptr = msg;
    g_vel_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("robot");
    auto state_publisher = g_node->create_publisher<RobotState>("~/state", 10);
    auto vel_subscriber = g_node->create_subscription<Twist>("~/cmd_vel", 10, onCmdVelSubscribed);
    std::thread([]{rclcpp::spin(g_node);}).detach();

    const auto WHEEL_DIAMETER = g_node->declare_parameter<double>("wheel_diameter", 0.1);
    const auto ENCODER_COUNT = g_node->declare_parameter<int>("encoder_count", 1000);
    std::srand(std::time(nullptr));
    RobotState state;
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        g_vel_mutex.lock();
        auto vel_ptr = g_vel_ptr;
        g_vel_mutex.unlock();

        state.odometry.twist.twist.linear.x = vel_ptr->linear.x;
        state.odometry.twist.twist.linear.y = vel_ptr->linear.y;
        state.odometry.twist.twist.linear.z = vel_ptr->linear.z;
        state.odometry.twist.twist.angular.x = vel_ptr->angular.x;
        state.odometry.twist.twist.angular.y = vel_ptr->angular.y;
        state.odometry.twist.twist.angular.z = vel_ptr->angular.z;
        state.odometry.pose.pose.position.x = std::rand() % 10; 
        state.odometry.pose.pose.position.y = std::rand() % 10; 
        state.odometry.pose.pose.position.z = std::rand() % 10; 
        state.odometry.pose.pose.orientation.w = std::rand() % 10;
        state.odometry.pose.pose.orientation.x = std::rand() % 10;
        state.odometry.pose.pose.orientation.y = std::rand() % 10;
        state.odometry.pose.pose.orientation.z = std::rand() % 10;
        state.imu.linear_acceleration.x = std::rand() % 10;
        state.imu.linear_acceleration.y = std::rand() % 10;
        state.imu.linear_acceleration.z = std::rand() % 10;
        state.imu.angular_velocity.x = std::rand() % 10;
        state.imu.angular_velocity.y = std::rand() % 10;
        state.imu.angular_velocity.z = std::rand() % 10;
        state.imu.orientation.w = std::rand() % 10;
        state.imu.orientation.x = std::rand() % 10;
        state.imu.orientation.y = std::rand() % 10;
        state.imu.orientation.z = std::rand() % 10;
        state.battery = std::rand() % 10;
        state.encoder.resize(2);
        state.encoder[0] = std::rand() % 10;
        state.encoder[1] = std::rand() % 10;
        state.rpm.resize(2);
        state.rpm[0] = 60.0 * M_PI * WHEEL_DIAMETER * state.encoder[0] / ENCODER_COUNT;
        state.rpm[1] = 60.0 * M_PI * WHEEL_DIAMETER * state.encoder[1] / ENCODER_COUNT;
        state.current.resize(2);
        state.current[0] = std::rand() % 10;
        state.current[1] = std::rand() % 10;
        state.emergency = (std::rand() % 10) > 5;

        state_publisher->publish(state);

        RCLCPP_INFO(g_node->get_logger(), "vel: [%lf %lf %lf][%lf %lf %lf]",
            vel_ptr->linear.x,
            vel_ptr->linear.y,
            vel_ptr->linear.z,
            vel_ptr->angular.x,
            vel_ptr->angular.y,
            vel_ptr->angular.z
        );
    }
    rclcpp::shutdown();
}