#include <software_training_assignment/cmd_vel_moving_turt_publisher.hpp>

using namespace std::chrono_literals;

namespace composition {

    cmd_vel_moving_turt_publisher::cmd_vel_moving_turt_publisher() : Node("cmd_vel_moving_turt_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&cmd_vel_moving_turt_publisher::timer_callback, this));
    }

    void cmd_vel_moving_turt_publisher::timer_callback() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1;
        message.angular.z = 1;
        RCLCPP_INFO(this->get_logger(), "Turtle1 moving in a circle");
        publisher_->publish(message);
    }
} // namespace composition

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<composition::cmd_vel_moving_turt_publisher>());
    rclcpp::shutdown();
    return 0;
}
