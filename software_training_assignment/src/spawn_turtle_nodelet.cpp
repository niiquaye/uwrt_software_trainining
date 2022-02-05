#include <software_training_assignment/spawn_turtle_nodelet.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtle_spawn_client");
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client =
        node->create_client<turtlesim::srv::Spawn>("spawn");


    for (auto &turtle : turtles) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle.name;
        request->x = turtle.x;
        request->y = turtle.y;
        request->theta = turtle.rad;

        while (!client->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Interrupted while waiting for service. Exiting!");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available after waiting");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turtles spawned!");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service kill");
        }
    } // for loop
    rclcpp::shutdown(); // need this or else will keep on executing callback -
    return 0;
} // int main


