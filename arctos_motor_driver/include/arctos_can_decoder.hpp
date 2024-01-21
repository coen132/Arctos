#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
class servo42_can_decoder : public rclcpp::Node {
  public:
    servo42_can_decoder();
    
  private:
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
    int node_id = 0;   
    const std::string DEFAULT_JOINT_STATES_TOPIC = "/joint_states";
    std::string from_can_bus_topic = "/from_can_bus";
    std::string joint_states_topic= servo42_can_decoder::DEFAULT_JOINT_STATES_TOPIC;

    void declare_all_params();
    void get_initial_params();
    void receive_can_frame(can_msgs::msg::Frame::SharedPtr msg);
    void parse_motor_angle(can_msgs::msg::Frame can_frame);

};
