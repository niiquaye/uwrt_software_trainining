#include "../include/software_training_assignment/turtle_distance_node.hpp"

using namespace composition;
turtle_distance_node::turtle_distance_node(const rclcpp::NodeOptions &options) : Node("turtle_distance_node") {
    //need to subscribe to pose for turtle 
    auto callback1 =  [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this -> turtle1_x = msg -> x;
        this -> turtle1_y = msg -> y;
    };
    auto callback2 =  [this](const  turtlesim::msg::Pose::SharedPtr msg) -> void {
        this -> turtle2_x = msg -> x;
        this -> turtle2_y = msg -> x;
    };
    auto subscription_1 = this -> create_subscription<turtlesim::msg::Pose>("moving_turtle/pose", 10, callback1);
    auto subscription_2 = this -> create_subscription<turtlesim::msg::Pose>("stationary_turtle/pose", 10, callback2);

    auto publisher_callback = [this]() -> void {
      float x{abs(this ->turtle1_x - this -> turtle2_x)};
      float y{abs(this ->turtle1_y - this -> turtle2_y)};
      float distance{turtleDistance()};
      auto msg = std::make_unique<software_training::msg::Software>();
      msg -> x = x;
      msg -> y = y;
      msg -> distance = distance;
      this -> publisher -> publish(std::move(msg));
    };

    publisher = this -> create_publisher<software_training::msg::Software>("/difference", 10);
    timer = this -> create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), publisher_callback);
}

float turtle_distance_node::turtleDistance() {
    float distance = sqrtf(pow((turtle1_x - turtle2_x), 2) + pow((turtle1_y - turtle2_y), 2));
    return distance;
}
