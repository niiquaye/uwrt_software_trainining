#include <memory>
#include <software_training/spawn_turtle_nodelet.hpp>

using namespace std::chrono_literals;

namespace composition {

spawn_turtle_nodelet::spawn_turtle_nodelet(const rclcpp::NodeOptions &options)
    : Node("spawn_turtle_nodelet", options) {

  // create client that makes a requets to '/spawn' service
  client = this->create_client<turtlesim::srv::Spawn>("/spawn");

  // create client callback
  timer = this->create_wall_timer(
      2s, std::bind(&spawn_turtle_nodelet::spawn_turtle, this));

  // fill up map with contents
  for (size_t i{0}; i < NUMBER_OF_TURTLES; i++) {
    turtle_description.insert({turtle_names[i], turtle_bio[i]});
  }
}

void spawn_turtle_nodelet::spawn_turtle() {

  if (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Service was interupted - EXIT");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for service - DNE - EXIT");
    return;
  }

  for (const std::string &name : turtle_names) {

    // create request
    std::unique_ptr<turtlesim::srv::Spawn::Request> request =
        std::make_unique<turtlesim::srv::Spawn::Request>();

    // fill in repsonse
    request->name = name;
    request->x = turtle_description[name].x_pos;
    request->y = turtle_description[name].y_pos;
    request->theta = turtle_description[name].rad;

    // create a callback to call client and because no 'spin()' is available
    auto callback =
        [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response)
        -> void {
      RCLCPP_INFO(this->get_logger(), "Turtle Created: %s",
                  response.get()->name.c_str());
      rclcpp::shutdown();
    };

    // send request
    auto result = client->async_send_request(std::move(request), callback);
  }
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::spawn_turtle_nodelet)
