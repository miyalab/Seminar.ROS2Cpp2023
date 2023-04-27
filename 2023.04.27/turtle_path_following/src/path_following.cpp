#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>

constexpr double MAX_LINEAR_VELOCITY = 1.0;
constexpr double MAX_ANGULAR_VELOCITY = 1.0;
constexpr double THRESH_THETA = 3.0 * M_PI / 180;

std::mutex g_mutex;
geometry_msgs::msg::Pose2D::SharedPtr g_location_ptr;
geometry_msgs::msg::Pose2D::SharedPtr g_target_ptr;
rclcpp::Node::SharedPtr g_node;

void onTargetSubscribed(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    g_mutex.lock();
    g_target_ptr = msg;
    g_mutex.unlock();
}

void onLocationSubscribed(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    g_mutex.lock();
    g_location_ptr = msg;
    g_mutex.unlock();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("path_following");
    auto location_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose2D>("/locator/location", 10, onLocationSubscribed);
    auto target_subscriber = g_node->create_subscription<geometry_msgs::msg::Pose2D>("/path_planning/target_pose", 10, onTargetSubscribed);
    auto vel_publisher = g_node->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);

    geometry_msgs::msg::Twist vel_msg;
    std::thread([]{rclcpp::spin(g_node);}).detach();
    for(rclcpp::WallRate loop(30); rclcpp::ok(); loop.sleep()){
        g_mutex.lock();
        auto location_ptr = g_location_ptr;
        auto target_ptr = g_target_ptr;
        g_mutex.unlock();
        if(!location_ptr || !target_ptr) continue;

        geometry_msgs::msg::Pose2D error;
        double error_r = std::hypot(error.x = target_ptr->x - location_ptr->x, error.y = target_ptr->y - location_ptr->y);
        // 上記，以下プログラムと同等
        // error.x = target_ptr->x - location_ptr->x;
        // error.y = target_ptr->y - location_ptr->y;
        // double error_r = std::hypot(error.x, error.y);

        error.theta = (error_r < 0.01)? target_ptr->theta: std::atan2(error.y, error.x) - location_ptr->theta;
        // 上記，以下プログラムと同等
        // double target_theta;
        // if(error_r < 0.01) target_theta = target_ptr->theta;
        // else target_theta = std::atan2(error.y, error.x);
        // error.theta = target_theta - location_ptr->theta;

        error.theta += 2 * M_PI * ((error.theta < -M_PI) - (error.theta > M_PI));
        // 上記，以下プログラムと同等
        // if(error.theta > M_PI)       error.theta -= 2*M_PI;
        // else if(error.theta < -M_PI) error.theta += 2*M_PI;

        error_r *= (-THRESH_THETA < error.theta && error.theta < THRESH_THETA);
        // 上記，以下プログラムと同等
        // if(std::abs(error.theta) > THRESH_THETA) error_r = 0;

        vel_msg.linear.x  = error_r 
                          +(MAX_LINEAR_VELOCITY - error_r) * (error_r > MAX_LINEAR_VELOCITY);
        vel_msg.angular.z = error.theta 
                          + (MAX_ANGULAR_VELOCITY - error.theta*(1 - 2*(error.theta < 0))) 
                            * ((error.theta > MAX_ANGULAR_VELOCITY) - (error.theta < -MAX_ANGULAR_VELOCITY));
        // 上記，以下プログラムと同等（min関数は中身でifが使用されている）
        // vel_msg.linear.x = std::min(error_r, MAX_LINEAR_VELOCITY);
        // if(error.theta >  MAX_ANGULAR_VELOCITY) error.theta =  MAX_ANGULAR_VELOCITY;
        // if(error.theta < -MAX_ANGULAR_VELOCITY) error.theta = -MAX_ANGULAR_VELOCITY;
        // vel_msg.angular.z = error.theta;

        vel_publisher->publish(vel_msg);
        RCLCPP_INFO(g_node->get_logger(), "[%lf, %lf]", vel_msg.linear.x, vel_msg.angular.z);
    }
}