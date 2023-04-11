#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <tutorial_interfaces/msg/int32_stamped.hpp>

rclcpp::Node::SharedPtr g_node = nullptr;
std::mutex g_mutex;
tutorial_interfaces::msg::Int32Stamped::SharedPtr g_msg_ptr;

void onNumberSubscribed(const tutorial_interfaces::msg::Int32Stamped::SharedPtr msg)
{
    g_mutex.lock();
    g_msg_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("subscriber");
    auto subscriber = g_node->create_subscription<tutorial_interfaces::msg::Int32Stamped>("number", 10, onNumberSubscribed);
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto msg_ptr = g_msg_ptr;
        g_mutex.unlock();
        if(msg_ptr){
            RCLCPP_INFO(g_node->get_logger(), "I subscribed: [%d.%09d] %d", 
                msg_ptr->header.stamp.sec, 
                msg_ptr->header.stamp.nanosec, 
                msg_ptr->data);
        }
    }
    g_node = nullptr;
    rclcpp::shutdown();
}
