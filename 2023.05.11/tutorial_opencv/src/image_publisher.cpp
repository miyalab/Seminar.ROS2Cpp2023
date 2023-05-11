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
    m_thread.release();
}

/**
 * @brief Execute method
 * 
 */
void ImagePublisher::run()
{
    RCLCPP_INFO(this->get_logger(), "%s has started. thread id = %0x", this->get_name(), std::this_thread::get_id());
    
    // Main loop
    for(rclcpp::WallRate loop(1); rclcpp::ok(); loop.sleep()){
        
    }

    RCLCPP_INFO(this->get_logger(), "%s has stoped.", this->get_name());
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROS2Tutorial::ImagePublisher)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------