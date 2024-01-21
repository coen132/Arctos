#include "arctos_can_encoder.hpp"
#include <iomanip>
#include <string>
#include <chrono>
using namespace std::chrono_literals;

servo42_can_encoder::servo42_can_encoder() : Node("can_encoder"){

    this->declare_all_params();
    this->get_initial_params();
    this->motor_angle_can_pub = this->create_publisher<can_msgs::msg::Frame>(this->motor_angle_can_topic, 1000);
        timer_ = this->create_wall_timer(
    5ms, std::bind(&servo42_can_encoder::request_motor_angle, this));
}

void servo42_can_encoder::declare_all_params() {

    // Apply node's namespace to the published topics
    this->declare_parameter("motor_angle_can_topic", DEFAULT_MOTOR_ANGLE_CAN_TOPIC);
}

void servo42_can_encoder::get_initial_params() {
    this->get_parameter("node_id", this->node_id);
    this->get_parameter("motor_angle_can_topic", this->motor_angle_can_topic);
}

void servo42_can_encoder::request_motor_angle() {

    can_msgs::msg::Frame request_angle_frame;
    request_angle_frame.dlc = 2;
    request_angle_frame.data = {48, 49, 0, 0, 0, 0, 0, 0};
    request_angle_frame.id = 1;
    motor_angle_can_pub->publish(request_angle_frame);
    
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<servo42_can_encoder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}