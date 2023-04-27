#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <turtlesim/msg/pose.hpp>


std::mutex g_mutex;
turtlesim::msg::Pose::SharedPtr g_pose_ptr;
rclcpp::Node::SharedPtr g_node;

void onPoseSubscribed(const turtlesim::msg::Pose::SharedPtr msg)
{
    g_mutex.lock();
    g_pose_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("locator");
    auto pose_subscriber = g_node->create_subscription<turtlesim::msg::Pose>("/robot/pose", 10, onPoseSubscribed);
    auto location_publisher = g_node->create_publisher<geometry_msgs::msg::Pose2D>("~/location", 10);

    geometry_msgs::msg::Pose2D location_msg;
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto pose_ptr = g_pose_ptr;
        g_mutex.unlock();
        if(!pose_ptr) continue;

        location_msg.x = pose_ptr->x;
        location_msg.y = pose_ptr->y;
        location_msg.theta = pose_ptr->theta;

        location_publisher->publish(location_msg);
        RCLCPP_INFO(g_node->get_logger(), "[%lf, %lf, %lf]", location_msg.x, location_msg.y, location_msg.theta);
    }
}