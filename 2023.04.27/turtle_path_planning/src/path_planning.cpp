#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

std::mutex g_mutex;
geometry_msgs::msg::Pose2D::SharedPtr g_location_ptr;
rclcpp::Node::SharedPtr g_node;

constexpr double target_point[][2] = {
    {0,0}, {4,0}, {4,4}, {0,4}
};

void onLocationSubscribed(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    g_mutex.lock();
    g_location_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("locator");
    auto location_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose2D>("/locator/location", 10, onLocationSubscribed);
    auto target_publisher = g_node->create_publisher<geometry_msgs::msg::Pose2D>("~/target_pose", 10);

    int index = 0;
    geometry_msgs::msg::Pose2D target_msg;
    target_msg.x = target_point[index][0] + 4;
    target_msg.y = target_point[index][1] + 4;
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto location_ptr = g_location_ptr;
        g_mutex.unlock();
        if(!location_ptr) continue;

        // 目標位置とのズレを判定
        if(std::hypot(target_msg.x - location_ptr->x, target_msg.y - location_ptr->y) <= 0.01){
            index *= (++index < 4);
            /** 上記は以下のコードと同等
             * i++;
             * if(i>=4) i=0;
             */
            target_msg.x = target_point[index][0] + 4;
            target_msg.y = target_point[index][1] + 4;
        }

        target_publisher->publish(target_msg);
        RCLCPP_INFO(g_node->get_logger(), "[%lf, %lf, %lf]", target_msg.x, target_msg.y, target_msg.theta);
    }
}