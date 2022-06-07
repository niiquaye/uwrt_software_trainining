#include "../include/software_training_assignment/clear_turtles_node.hpp"

using namespace composition;

clear_turtles_node::clear_turtles_node(const rclcpp::NodeOptions &options) : Node("clear_turtles_node") {
    
    client = this -> create_client<turtlesim::srv::Kill>("kill");

    timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), std::bind(&clear_turtles_node::kill, this));
   
}

void clear_turtles_node::kill() {
    //kill all the nodes by sending a request to each name
    for(auto turtle : nodes) {
        //send a kill request to turtlesim for each turtle name
        //first check if the turtle exists
        //then delete the turtle
        //make requests i guess
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        //need a callback function

        auto callback = [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void {
            std::cout << "finished" << std::endl;
            rclcpp::shutdown();
        };
        
        client -> async_send_request(request, callback);
    }

}