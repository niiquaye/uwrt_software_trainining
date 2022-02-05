#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

struct turtle_info {
    float x;
    float y;
    float rad;
    std::string name;
};
  
std::vector<turtle_info> turtles {
    {5, 5, 0, "stationary_turtle"},
    {25, 10, 0, "moving_turtle"}   
};
 
