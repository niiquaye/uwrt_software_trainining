#include <rclcpp>
#include <cstdlib>
#include <cmath>


class turtle_distance_node : public rclcpp::Node {
    public:
        turtle_distance_node(const rclcpp::NodeOptions &options);
    private:
        rclcpp::Subscription<std_msg::nsg::String> subscription;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::timer::wallTimer timer_cb;
        double turtleDistance(std::pair<double, double> turtle1, std::pair<double, double> turtle2);
}
