#include "circular_turtles_node.hpp"

circular_turtle::circular_turtle() {
    //if this gets called then 

    //create a service
    client = create_client<turtlesim::srv::Spawn>("/spawn");
    timer_cb = create_wall_timer(2s, std::bind(spawn, this)); //turns the function into a function pointer
}

void spawn() {

}