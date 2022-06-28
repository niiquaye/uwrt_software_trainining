#include "../include/software_training_assignment/spawn_turtles_node.hpp"

    
spawn_turtles::spawn_turtles(const rclcpp::NodeOptions &options) : Node("spawn_turtles_node") {
    timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), std::bind(&spawn_turtles::spawn, this));
}

void spawn_turtles::spawn() {
    //make stationary turtle
    client = this -> create_client<turtlesim::srv::Spawn> ("spawn");
    auto request1 = std::make_shared<turtlesim::srv::Spawn>();
    request1 -> x = 25;
    request1 -> y = 25;
    request1 -> theta = 0;
    request1-> name = "moving_turtle";
    auto request2 = std::make_shared<turtlesim::srv::Spawn>();
    request2->x = 25;
    request2->y = 25;
    request2->theta = 0;
    request2->name = "stationary_turtle";

    auto callbackFunction1 = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void{
        std::cout << "sent moving_turtle"
    };
    auto callbackFunction2 = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void{
        std::cout << "send stationary turtle" << std::endl;
    };
    client -> async_send_request(request1, callbackFunction1);
    client -> async_send_request(request2, callbackFunction2);

    

}