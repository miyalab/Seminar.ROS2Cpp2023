#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <mutex>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

rclcpp::Node::SharedPtr g_node = nullptr;
std::mutex g_mutex;
geometry_msgs::msg::Point::SharedPtr g_point_ptr;
geometry_msgs::msg::Pose::SharedPtr g_pose_ptr;

void onPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    g_mutex.lock();
    g_pose_ptr = msg;
    g_mutex.unlock();
}

void onPointSubscribed(const geometry_msgs::msg::Point::SharedPtr msg)
{
    g_mutex.lock();
    g_point_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("subscriber");
    auto pose_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose>("publisher/pose", 10, onPoseSubscribed);
    auto point_subscriber = g_node->create_subscription<geometry_msgs::msg::Point>("publisher/point", 10, onPointSubscribed);

    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto point_ptr = g_point_ptr;
        auto pose_ptr = g_pose_ptr;
        g_mutex.unlock();

        if(point_ptr) RCLCPP_INFO(g_node->get_logger(), "I subscribed point: [%lf, %lf, %lf]", point_ptr->x, point_ptr->y, point_ptr->z);
        if(pose_ptr)  RCLCPP_INFO(g_node->get_logger(), "I subscribed pose : [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", 
            pose_ptr->position.x, pose_ptr->position.y, pose_ptr->position.z, 
            pose_ptr->orientation.w, pose_ptr->orientation.x, pose_ptr->orientation.y, pose_ptr->orientation.z
        );
    }
    g_node = nullptr;
    rclcpp::shutdown();
}
