#include <cstdlib>
#include <rclcpp>
#include <turtlesim/srv/Spawn>

class spawn_turtles : public rclcpp::Node {
private:
    /* data */

    spawn_turtles(const rclcpp::NodeOptions &options);
    rclcpp::Client<turtlesim::srv::Spawn>* client;
    rclcpp::Client<turtlesim::srv::Spawn>::sharedRequest* sharedRequest;
    rclcpp::timer::WallTimer timer_cb;
    void spawn();
    
public:
    
};

