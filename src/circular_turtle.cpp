#include "circular_turtles_node.hpp"

circular_turtle::circular_turtle(const rclcpp::NodeOptions &options) {
    //if this gets called then 

    //create a service
    client = create_client<turtlesim::srv::Spawn>("/spawn");
    
    timer_cb = create_wall_timer(2s, std::bind(spawn, this)); //turns the function into a function pointer
}

void spawn() {
    //call the service
    sharedReq = std::make_shared<rclcpp::Client<turtlesim::srv::Spawn>::SharedRequest> (0, 0, 0, "turtle1");
    auto callback = [] -> {
        std::cout << "made turtle1";
    }
    client.send_async_request(sharedReq, callback);
    
    //moving a turtle needs a pub/sub relationship
}