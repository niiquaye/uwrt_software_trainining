#include <software_training_assignment/turtle_publisher.hpp>

namespace composition {

    using namespace std::chrono_literals;
    using std::placeholders::_1;

    TurtlePublisher::TurtlePublisher() : Node("turtle_publisher") {
        publisher_ =
            this->create_publisher<software_training_assignment::msg::Software>("/difference", 10);

        moving_sub = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", 10, std::bind(&TurtlePublisher::moving_callback, this, _1));
        stationary_sub = this->create_subscription<turtlesim::msg::Pose>("/stationary_turtle/pose", 10, std::bind(&TurtlePublisher::stationary_callback, this, _1));

        timer_ = this->create_wall_timer(1s, std::bind(&TurtlePublisher::publish, this));
    }

    void TurtlePublisher::moving_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        x_moving = msg->x;
        y_moving = msg->y;
    }

    void TurtlePublisher::stationary_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        x_stationary = msg->x;
        y_stationary = msg->y;
    }

    void TurtlePublisher::publish() {
        auto msg = software_training_assignment::msg::Software();
        // compute absolute difference in coordinates
        double position_x{abs(this->x_stationary- this->x_moving)};
        double position_y{abs(this->y_stationary- this->y_moving)};

        // create message to publish
        msg.x_pos = position_x;
        msg.y_pos = position_y;
        // compute distance using trig
        msg.distance = sqrt((position_x * position_x) + (position_y * position_y));

        this->publisher_->publish(msg);
    }
} // namespace composition

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<composition::TurtlePublisher>());
    rclcpp::shutdown();

    return 0;
}
