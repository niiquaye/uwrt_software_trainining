#include <turtle_distance_node.hpp>

turtle_distance_node::turtle_distance_node(const rclcpp::NodeOptions &options) {


    //need to subscribe to pose for turtle 
    auto callback1 =  [const std_msgs::msg::String::SharedPtr msg] -> {
        std::cout << msg;
    }
    subscription_1 = this -> create_subscription<std_msgs::msg::String>(
      "moving_turtle/pose", 10, std::bind(callback1, this));

    subscription_2 = this -> create_subscription<std_msgs::msg::String>(
      "stationary_turtle/pose", 10, std::bind(callback1, this));


    double distance = turtleDistance(subscription_1, subscription_2)

    publisher = this -> create_publisher<std_msgs::msg::String>(distance, subscription_1, subscription_2, 10);
}

double turtle_distance_node::turtleDistance(std::pair<double, double> turtle1, std::pair<double, double> turtle2) {
    double distance = sqrt((turtle1.first - turtle2.first) ^ 2 + (turtle1.second - turtle2.second) ^2);
    return distance;
}
