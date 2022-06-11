#include "reset_turtle_node.hpp"

reset_turtle_node::reset_turtle_node(const rclcpp::NodeOptions &options) {
    node = rclcpp::make_shared("reset_moving_turtle");
    service = node -> create_service<<std_srvs::srv::Empty>("reset_moving_turtle", &reset_moving_turtle) //pass it a reference to a funciton
}
void reset_moving_turtle() {
    //call turtlesim teleport absolute
    timer_cb = create_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), std::bind(&reset, this));
}

void reset() {
    sharedPtr = std::make_shared<rclcpp::Client<rclcpp::srv::TeleportAbsolute>>::SharedPtr>(25, 25, 0, "moving_turtle"); // how do i interact with turtleX lol the documentation doesn't take a name as  input
    auto successCallback = [this]() -> { 
        std::cout << "finished";
    }
    auto failureCallback = [this]() -> {
        std::cout << "Failure";
    }
    client -> send_async_request(sharedPtr, successCallback, failureCallback);
}
