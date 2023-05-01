#include <thread>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// 以下，クラス定義
namespace ROS2TutorialComponent{
class Publisher: public rclcpp::Node{
public:
    Publisher(rclcpp::NodeOptions options);
    virtual ~Publisher();
private:
    // publisher関連
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr value_publisher;

    std::unique_ptr<std::thread> thread;    
    void run();
};
}

// 以下，メソッド定義
namespace ROS2TutorialComponent{
Publisher::Publisher(rclcpp::NodeOptions options) : rclcpp::Node("publisher", options)
{
    this->value_publisher = this->create_publisher<std_msgs::msg::Int32>("value", 10);

    this->thread = std::make_unique<std::thread>(&Publisher::run, this);
    this->thread->detach();
}

Publisher::~Publisher()
{
    this->thread.release();
}

void Publisher::run()
{
    int cnt = 0;
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        auto msg = std::make_unique<std_msgs::msg::Int32>();
        msg->data = cnt++;
        RCLCPP_INFO_STREAM(this->get_logger(), "publish:" << std::hex << reinterpret_cast<std::uintptr_t>(msg.get()));
        this->value_publisher->publish(std::move(msg));
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROS2TutorialComponent::Publisher)