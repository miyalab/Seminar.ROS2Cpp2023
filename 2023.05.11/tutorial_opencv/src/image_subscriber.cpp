//-----------------------------
// include
//-----------------------------
// STL
#include <memory>
#include <thread>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>

//-----------------------------
// Namespace & using
//-----------------------------

//-----------------------------
// class
//-----------------------------
namespace ROS2Tutorial {
/**
 * @brief Component Definition
 * 
 */
class ImageSubscriber: public rclcpp::Node {
public:
    ImageSubscriber(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ImageSubscriber();
private:
    sensor_msgs::msg::Image::SharedPtr image_ptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    std::mutex image_mutex;
    void onImageSubscribed(const sensor_msgs::msg::Image::SharedPtr msg);

    std::unique_ptr<std::thread> thread;
    void run();
};
}

//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace ROS2Tutorial{
/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
ImageSubscriber::ImageSubscriber(rclcpp::NodeOptions options) : rclcpp::Node("image_subscriber", options)
{
    // Using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    // Initialize parameters
    RCLCPP_INFO(this->get_logger(), "Initialize parameters...");
    RCLCPP_INFO(this->get_logger(), "Complete! Parameters were initialized.");

    // Initialize subscriber
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers...");
    this->image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("~/image", 10, std::bind(&ImageSubscriber::onImageSubscribed, this, _1));
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&ImageSubscriber::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ImageSubscriber::~ImageSubscriber()
{
    this->thread.release();
}

void ImageSubscriber::onImageSubscribed(const sensor_msgs::msg::Image::SharedPtr msg)
{
    this->image_mutex.lock();
    this->image_ptr = msg;
    this->image_mutex.unlock();
}

/**
 * @brief Execute method
 * 
 */
void ImageSubscriber::run()
{
    // Main loop
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        this->image_mutex.lock();
        auto ros_image = this->image_ptr;
        this->image_mutex.unlock();
        if(!ros_image) continue;

        RCLCPP_INFO(this->get_logger(), "subsribed");
        auto cv_img = cv_bridge::toCvShare(ros_image, ros_image->encoding);
        RCLCPP_INFO_STREAM(this->get_logger(), "address: " << std::hex << ros_image.get());
        cv::imshow("debug", cv_img->image);
        cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROS2Tutorial::ImageSubscriber)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------