#ifndef CMD_VEL_MOVING_TURT_PUBLISHER_HPP_
#define CMD_VEL_MOVING_TURT_PUBLISHER_HPP_ 

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition {

    class cmd_vel_moving_turt_publisher : public rclcpp::Node {

        public:
            cmd_vel_moving_turt_publisher();

        private:
            void timer_callback();

            // publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

            // callback timer
            rclcpp::TimerBase::SharedPtr timer_;
    };
}


#endif //  CMD_VEL_MOVING_TURT_PUBLISHER_HPP_
