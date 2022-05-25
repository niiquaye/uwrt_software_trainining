#include <cstdlib>
#include <rclcpp>
#include <turtlesim/srv/TeleportAbsolute>

class reset_turtle_node : public rclcpp::Node {
    public:
        reset_turtle_node(const rclcpp::NodeOptions &options);
    private:
        //make a service.... 
        //how do i do that lmao
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
        rclcpp::timer::wallTimer timer;
        rclcpp::Client<rclcpp::srv::TeleportAbsolute> client;
        rclcpp::Client<rclcpp::srv::TeleportAbsolute>::SharedPtr sharedPtr;

} 