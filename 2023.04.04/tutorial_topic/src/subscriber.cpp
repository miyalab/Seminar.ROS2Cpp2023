#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <mutex>

#include <std_msgs/msg/int32.hpp>

rclcpp::Node::SharedPtr g_node = nullptr;
std::mutex g_mutex;
std_msgs::msg::Int32::SharedPtr g_msg_ptr;

void onNumberSubscribed(const std_msgs::msg::Int32::SharedPtr msg)
{
    g_mutex.lock();
    g_msg_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("subscriber");
    auto subscriber = g_node->create_subscription<std_msgs::msg::Int32>("number", 10, onNumberSubscribed);
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto msg_ptr = g_msg_ptr;
        g_mutex.unlock();
        if(msg_ptr) RCLCPP_INFO(g_node->get_logger(), "I subscribed: %d", msg_ptr->data);
    }
    g_node = nullptr;
    rclcpp::shutdown();
}
