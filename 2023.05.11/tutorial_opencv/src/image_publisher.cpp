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
class ImagePublisher: public rclcpp::Node {
public:
    ImagePublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~ImagePublisher();
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;

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
ImagePublisher::ImagePublisher(rclcpp::NodeOptions options) : rclcpp::Node("image_publisher", options)
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
    RCLCPP_INFO(this->get_logger(), "Complete! Subscribers were initialized.");

    // Initialize publisher
    RCLCPP_INFO(this->get_logger(), "Initialize publishers...");
    this->image_publisher = this->create_publisher<sensor_msgs::msg::Image>("~/image", 10);
    RCLCPP_INFO(this->get_logger(), "Complete! Publishers were initialized.");

    // Initialize Service-Server
    RCLCPP_INFO(this->get_logger(), "Initialize service-servers...");
    RCLCPP_INFO(this->get_logger(), "Complete! Service-servers were initialized.");

    // Initialize Service-Client 
    RCLCPP_INFO(this->get_logger(), "Initialize service-clients...");
    RCLCPP_INFO(this->get_logger(), "Complete! Service-clients were initialized.");

    // Main loop processing
    this->thread = std::make_unique<std::thread>(&ImagePublisher::run, this);
    this->thread->detach();
}

/**
 * @brief Destroy the class object
 * 
 */
ImagePublisher::~ImagePublisher()
{
    this->thread.release();
}

/**
 * @brief Execute method
 * 
 */
void ImagePublisher::run()
{
    // カメラのOpen
    cv::VideoCapture capture(0);
    if(!capture.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Capture device not found!");
        return;
    }

    // Main loop
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        auto ros_image = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage cv_image;
        cv_image.header.frame_id = "capture";
        cv_image.header.stamp = this->now();
        capture >> cv_image.image;
        cv_image.toImageMsg(*ros_image.get());
        image_publisher->publish(std::move(ros_image));
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROS2Tutorial::ImagePublisher)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------