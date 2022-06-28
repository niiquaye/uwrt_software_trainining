#include "../include/software_training_assignment/moving_turtle_action_node.hpp"


using namespace composition;
moving_turtle_action_node::moving_turtle_action_node() : Node("moving_turtle_action_node") {
    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/moving_turtle/cmd_vel", rclcpp::QoS(QUEUE));

    auto subscriber_callback =
      [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
    this->moving_turtle_action_server::x = msg->x;
    this->moving_turtle_action_server::y = msg->y;
    this->moving_turtle_action_server::theta = msg->theta;
    this->moving_turtle_action_server::linear_velocity = msg->linear_velocity;
    this->moving_turtle_action_server::angular_velocity = msg->angular_velocity;
  };


  // create subscriber
  this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
      "/moving_turtle/pose", QUEUE, subscriber_callback);

  // create action server
  this->action_server =
      rclcpp_action::create_server<software_training::action::Software>(
          this, "moving_turtle_action_server",
          std::bind(&moving_turtle_action_server::handle_goal, this, _1, _2),
          std::bind(&moving_turtle_action_server::handle_cancel, this, _1),
          std::bind(&moving_turtle_action_server::handle_accepted, this, _1)

      );
}


rclcpp_action::GoalResponse moving_turtle_action_node::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const software_training_assignment::action::Software::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse moving_turtle_action_node::handle_cancel(
    const std::shared_ptr<GoalHandleActionServer> goal_handle
) {
        return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandleActionserver> goal_handle) {
    std::thread{std::bind(&moving_turtle_action_server::execute, this, _1),
              goal_handle}
      .detach();
}

void moving_turtle_action_node::execute(const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto start_time = this -> now();
    
    float lin_x{0};
    float lin_y{0};
    float lin_z{0};

  
    float ang_x{0};
    float ang_y{0};
    float ang_z{0};

    while(rclcpp::ok() && (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y ||
          lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x ||
          ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z)) {

            if (goal_handle->is_canceling()) {
    
                rclcpp::Time curr_time = this->now();
                rclcpp::Duration time = curr_time - start_time;
                long int duration{time.nanoseconds()};
                result->duration = duration;

                goal_handle->canceled(std::move(result));
                return;


            }
            auto message = std::make_unique<geometry_msgs::msg::Twist>();
            message -> linear.x = (lin_x < goal -> linear_pos.x) ? ++lin_x : lin_x;
            message -> lienar.y = (lin_y < goal -> linear_pos.y) ? ++lin_y : lin_y;
            message -> linear.z = (lin_z < goal -> linear_pos.z) ? ++lin_z : liny_z;
            message -> angular.x = (ang_x < goal -> angular_pos.x) ? ++ang_x : ang_x;
            message -> angular.y = (ang_y < goal -> angular_pos.y) ? ++ang_y : ang_y;
            message -> angular.z = (ang_z < goal -> angular_pos.z) ? ++ang_z : ang_z;


            this->publisher->publish(std::move(message));

            auto feedback = std::make_unique<turtlesim::msg::Pose>();

            feedback -> x = this->moving_turtle_action_server::x - lin_x;
            feedback -> y = this->moving_turtle_action_server::y - lin_y;

             float theta{0};

            // scope this stuff cause we dont need it afterwards
            {

            float x1{lin_x}, x2{lin_y}, x3{lin_z};

            float magnitude{static_cast<float>(sqrt((x1 * x1) + (x2 * x2) + (x3 * x3)))};

            theta = acos(x3 / magnitude);
            }

            feedback -> theta = this -> moving_turtle_action_server::theta - theta;
          
            goal_handle->publish_feedback(std::move(feedback));

            cycle_rate.sleep(1); 
        }

    
    if (rclcpp::ok()) {
      result->sequence = sequence;
      
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      rclcpp::Time end = this->now();  
      rclcpp::Duration duration = end - start_time; 
        long int res_time{duration.nseconds()};    

        // fill in result
        result->duration = res_time;
        goal_handle->succeed(std::move(result));

    }
}

