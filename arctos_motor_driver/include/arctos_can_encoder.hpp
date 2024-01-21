#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

class servo42_can_encoder : public rclcpp::Node {
  public:
    servo42_can_encoder();
    
  private:
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr motor_angle_can_pub;
    const std::string DEFAULT_MOTOR_ANGLE_CAN_TOPIC = "/to_can_bus";   
    int node_id = 0;   

    std::string motor_angle_can_topic= servo42_can_encoder::DEFAULT_MOTOR_ANGLE_CAN_TOPIC;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;   

    void declare_all_params();
    void get_initial_params();
    void request_motor_angle();
};
