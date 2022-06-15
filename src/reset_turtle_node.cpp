#include "../include/software_training_assignment/reset_turtle_node.hpp"

using namespace composition;

reset_turtle_node::reset_turtle_node(const rclcpp::NodeOptions &options) {
    client_ = this -> create_client<turtlesim::srv::TeleportAbsolute>("reset_moving_turtle");
    service_ = this -> create_service<std_srvs::srv::Empty>("reset_moving_turtle", &reset_moving_turtle); //pass it a reference to a funciton
}
void reset_turtle_node::reset_moving_turtle() {
    //call turtlesim teleport absolute
    timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), std::bind(&reset_turtle_node::reset, this));
}

void reset_turtle_node::reset() {
    auto request = std::make_shared<rclcpp::srv::TeleportAbsolute>(25, 25, 0, "moving_turtle"); // how do i interact with turtleX lol the documentation doesn't take a name as  input
    auto callback = [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture response) -> { 
        response.wait();
        //send response based on shared future state
        if(response.get().)
       rclcpp::shutdown();
    };
    client_ -> async_send_request(request, callback);
}
