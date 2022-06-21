#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

class spawn_turtles : public rclcpp::Node {
private:
    /* data */
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
    
    rclcpp::TimerBase::SharedPtr timer_;
    void spawn();
    
public:
    spawn_turtles(const rclcpp::NodeOptions &options);
    
};

