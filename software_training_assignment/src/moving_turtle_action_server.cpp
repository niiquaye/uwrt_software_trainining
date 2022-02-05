#include <software_training_assignment/moving_turtle_action_server.hpp>

namespace composition {

    MovingTurtleActionServer::MovingTurtleActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("moving_turtle_action_server", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Software>(
                this,
                "moving_turtle_action_server",
                std::bind(&MovingTurtleActionServer::handle_goal, this, _1, _2),
                std::bind(&MovingTurtleActionServer::handle_cancel, this, _1),
                std::bind(&MovingTurtleActionServer::handle_accepted, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/moving_turtle/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/cmd_vel", 10, std::bind(&MovingTurtleActionServer::subscriber_callback, this, _1));
    }

    rclcpp_action::GoalResponse MovingTurtleActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Software::Goal> goal) {
        (void)uuid; // not needed - prevents compiler warnings/errors -Wall flag
        RCLCPP_INFO(this->get_logger(), "Goal Received");
        RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f Z:%f", goal->linear_pos.x, goal->linear_pos.y, goal->linear_pos.z);
        RCLCPP_INFO(this->get_logger(), "angular X:%f Y:%f Z:%f", goal->angular_pos.x, goal->angular_pos.y, goal->angular_pos.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MovingTurtleActionServer::handle_cancel(const std::shared_ptr<GoalHandleSoftware> goal_handle) {
        (void)goal_handle; // not needed
        RCLCPP_INFO(this->get_logger(), "Recieved Request to cancel goal!");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MovingTurtleActionServer::handle_accepted(const std::shared_ptr<GoalHandleSoftware> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MovingTurtleActionServer::execute, this, _1), goal_handle}.detach();
    }

    void MovingTurtleActionServer::execute(const std::shared_ptr<GoalHandleSoftware> goal_handle) {
        rclcpp::Time start_time = this->now(); // get the current time

        RCLCPP_INFO(this->get_logger(), "Excuting Goal");

        rclcpp::Rate cycle_rate{1}; // set up frequency for goal execution

        // acquire the goal to be executed
        const auto goal = goal_handle->get_goal();

        // create feedback
        auto feedback = std::make_shared<Software::Feedback>();

        // make result
        auto result = std::make_shared<Software::Result>();

        // create reference to feedback for ease of use
        float &curr_x = feedback->x_pos;
        float &curr_y = feedback->y_pos;
        float &curr_theta = feedback->theta_pos;

        // keep track of linear feedback
        float lin_x{0};
        float lin_y{0};
        float lin_z{0};

        // keep track of angular feedback
        float ang_x{0};
        float ang_y{0};
        float ang_z{0};

        // heavy lifting
        while (rclcpp::ok() &&
                (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y ||
                 lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x ||
                 ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z)) {

            // check if goal has been canceled
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");

                // get the time it has taken thus far and update result
                rclcpp::Time curr_time = this->now();
                rclcpp::Duration time = curr_time - start_time;
                long int duration{time.nanoseconds()};
                result->duration = duration;

                goal_handle->canceled(result);
                return;
            }

            // publish to '/moving_turtle/cmd_vel' topic to move the turtle

            // create message
            auto message_cmd_vel = geometry_msgs::msg::Twist();

            // fill message contents
            // if lin or ang distance points are already at target do not increment any
            // more just keep it the same hence the ternary operator
            message_cmd_vel.linear.x = (lin_x < goal->linear_pos.x) ? lin_x++ : lin_x;
            message_cmd_vel.linear.y = (lin_y < goal->linear_pos.y) ? lin_y++ : lin_y;
            message_cmd_vel.linear.z = (lin_z < goal->linear_pos.z) ? lin_z++ : lin_z;
            message_cmd_vel.angular.x =
                (ang_x < goal->angular_pos.x) ? ang_x++ : ang_x;
            message_cmd_vel.angular.y =
                (ang_y < goal->angular_pos.y) ? ang_y++ : ang_y;
            message_cmd_vel.angular.z =
                (ang_z < goal->angular_pos.z) ? ang_z++ : ang_z;

            // publish message
            this->publisher_->publish(message_cmd_vel);

            // now compute feedback
            curr_x = this->x - lin_x;
            curr_y = this->y - lin_y;

            /********* MATH TO FIND THETA ************
             * theta = acos(z_axis_component/magnitude of vector)
             *
             *
             ********************************************/
            float theta{0};

            // scope this stuff cause we dont need it afterwards
            {

                float x1{lin_x}, x2{lin_y}, x3{lin_z};

                float magnitude{static_cast<float>(sqrt((x1 * x1) + (x2 * x2) + (x3 * x3)))};

                theta = acos(x3 / magnitude);
            }

            curr_theta = this->theta - theta;

            // publish feedback
            goal_handle->publish_feedback(feedback);

            cycle_rate.sleep(); // control the rate at which the loop, loops through
        }

        // if goal is done
        if (rclcpp::ok()) {

            rclcpp::Time end = this->now();               // get end time
            rclcpp::Duration duration = end - start_time; // compute time taken
            long int res_time{duration.nanoseconds()};    // should be uint64_t

            // fill in result
            result->duration = res_time;

            // set the result
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Finish Executing Goal");

        }
    }

    void MovingTurtleActionServer::subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        this->x= msg->x;
        this->y = msg->y;
        this->theta = msg->theta;
        this->linear_velocity = msg->linear_velocity;
        this->angular_velocity = msg->angular_velocity;
    }
} // namespace composition

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MovingTurtleActionServer)
