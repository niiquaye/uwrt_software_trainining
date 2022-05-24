#include "clear_turtles_node.hpp";

using namespace composition;

clear_turtles_node::clear_turtles_node(const rclcpp::NodeOptions &options) {
    client = create_client<turtlesim::srv::Kill>("/kill");

    timer_cb = create_wall_timer(2s, std::bind(kill, this));
}

void clear_turtles_node::kill() {
    //kill all the nodes by sending a request to each name
    for(auto turtle : nodes) {
        //send a kill request to turtlesim for each turtle name
        //first check if the turtle exists
        //then delete the turtle
        //make requests i guess
        sharedReq = std::make_shared<rclcpp::Client<turtlesim::srv::Kill>::SharedRequest> (turtle);
        //need a callback function

        auto callback = []{
            std::cout << "finished" << std::endl;
        }
        
        client -> async_send_request(sharedReq, callback);
    }

}