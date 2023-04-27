#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::mutex g_mutex;
geometry_msgs::msg::Pose2D::SharedPtr g_location_ptr;
geometry_msgs::msg::Pose2D::SharedPtr g_target_ptr;
rclcpp::Node::SharedPtr g_node;

void onTargetSubscribed(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    g_mutex.lock();
    g_target_ptr = msg;
    g_mutex.unlock();
}

void onLocationSubscribed(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    g_mutex.lock();
    g_location_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("path_following");
    auto location_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose2D>("/locator/location", 10, onLocationSubscribed);
    auto target_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose2D>("/path_planning/target_pose", 10, onTargetSubscribed);
    auto vel_publisher = g_node->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

    geometry_msgs::msg::Twist vel_msg;
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto location_ptr = g_location_ptr;
        auto target_ptr = g_target_ptr;
        g_mutex.unlock();
        if(!location_ptr || !target_ptr) continue;

        double error_x = target_ptr->x - location_ptr->x;
        double error_y = target_ptr->y - location_ptr->y;
        double error_r = std::hypot(error_x, error_y);

        double target_theta;
        if(error_r < 0.01) target_theta = target_ptr->theta;
        else target_theta = std::atan2(error_y, error_x);

        double error_theta = target_theta - location_ptr->theta;
        if(error_theta > M_PI) error_theta -= 2*M_PI;
        else if(error_theta < -M_PI) error_theta += 2*M_PI;
        if(std::abs(error_theta) > 3.0 * M_PI / 180) error_r = 0;

        vel_msg.linear.x = std::min(error_r, 1.0);
        if(error_theta > 1)  error_theta = 1;
        if(error_theta < -1) error_theta = -1;
        vel_msg.angular.z = error_theta;

        vel_publisher->publish(vel_msg);
        RCLCPP_INFO(g_node->get_logger(), "[%lf, %lf]", vel_msg.linear.x, vel_msg.angular.z);
    }
}