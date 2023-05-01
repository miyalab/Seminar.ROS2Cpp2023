#include <thread>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// 以下，クラス定義
namespace ROS2TutorialComponent{
class Subscriber: public rclcpp::Node{
public:
    Subscriber(rclcpp::NodeOptions options);
    virtual ~Subscriber();
private:
    // Subscriber関連
    std::mutex value_mutex;
    std_msgs::msg::Int32::SharedPtr value_ptr;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr value_subscriber;
    void onValueSubscribed(const std_msgs::msg::Int32::SharedPtr msg);

    std::unique_ptr<std::thread> thread;    
    void run();
};
}

// 以下，メソッド定義
namespace ROS2TutorialComponent{
Subscriber::Subscriber(rclcpp::NodeOptions options) : rclcpp::Node("subscriber", options)
{
    using std::placeholders::_1;

    this->value_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "value", 10, std::bind(&Subscriber::onValueSubscribed, this, _1));

    this->thread = std::make_unique<std::thread>(&Subscriber::run, this);
    this->thread->detach();
}

Subscriber::~Subscriber()
{
    this->thread.release();
}

void Subscriber::onValueSubscribed(const std_msgs::msg::Int32::SharedPtr msg)
{
    this->value_mutex.lock();
    this->value_ptr = msg;
    this->value_mutex.unlock();
    RCLCPP_INFO_STREAM(this->get_logger(), "subscribed:" << std::hex << reinterpret_cast<std::uintptr_t>(this->value_ptr.get()));
}

void Subscriber::run()
{
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        this->value_mutex.lock();
        auto value = this->value_ptr;
        this->value_ptr = nullptr;
        this->value_mutex.unlock();
        if(!value) continue; // value_ptrがnull -> 未受信
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROS2TutorialComponent::Subscriber)