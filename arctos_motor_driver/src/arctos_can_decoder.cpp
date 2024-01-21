#include "arctos_can_decoder.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iomanip>
#include <string>

servo42_can_decoder::servo42_can_decoder() : Node("can_decoder") {
    // Declare all initial parameters and set them to default values
    this->declare_all_params();
    this->get_initial_params();

    // Create subscribers
    this->can_sub = this->create_subscription<can_msgs::msg::Frame>(
        from_can_bus_topic, 10, std::bind(&servo42_can_decoder::receive_can_frame, this, std::placeholders::_1));

    // Create publishers
    this->joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>(this->joint_states_topic, 1000);
    RCLCPP_INFO(this->get_logger(), "Waiting for CAN messages for node ID %i", this->node_id);
}

void servo42_can_decoder::declare_all_params() {
    this->declare_parameter("node_id", this->node_id);
    this->declare_parameter("from_can_bus_topic", this->from_can_bus_topic);

    // Apply node's namespace to the published topics
    this->declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC);
}

void servo42_can_decoder::get_initial_params() {
    this->get_parameter("node_id", this->node_id);
    this->get_parameter("from_can_bus_topic", this->from_can_bus_topic);
    this->get_parameter("joint_states_topic", this->joint_states_topic);
}

// double calculate_angle(const can_msgs::msg::Frame& can_frame){

//     std::cout << "Inside calculate_angle" << std::endl;
//     if (can_frame.data.size() < 8) {
//         std::cout << "Invalid data size in calculate_angle" << std::endl;
//         return 0;  // or some default value
//     }
//     int angle; //degrees
//     uint16_t frame_data[8];
//     for (int i = 0; i < 8; ++i) {
//         frame_data[i] = can_frame.data[i];
//     }
//     double value = 0;
//     for (int i = 6; i < 8; ++i) {
//         value += frame_data[i];
//     }
//     //0x400hex or 16384 decimal: full circle
//     angle = value / 16384 * 360;
//     return angle;
// }

void servo42_can_decoder::parse_motor_angle(can_msgs::msg::Frame can_frame) {

    sensor_msgs::msg::JointState joint_msg;
    // Store metadata
    // std::cout << "Angle received \n";
    joint_msg.header.stamp = can_frame.header.stamp;

    // joint_msg.name.emplace_back("joint1");
    // joint_msg.name.emplace_back("joint2");
    // joint_msg.name.emplace_back("joint3");
    // joint_msg.name.emplace_back("joint4");
    // joint_msg.name.emplace_back("joint5");
    // joint_msg.name.emplace_back("joint6");
    std::cout << "Inside parse_can frame1" << std::endl;
    //double angle = calculate_angle(can_frame);
    //joint_msg.position[0] = angle;
    //std::cout << "Setting angle to:" << angle << "\n";
    // Various enable state bits in byte 1, split into individual bits
    //uint8_t status_byte = (uint8_t)(msg->data[1]);
    // Publish the message immediately
    this->joint_states_pub->publish(joint_msg);
}

void servo42_can_decoder::receive_can_frame(can_msgs::msg::Frame::SharedPtr msg) {

    std::cout << "Inside receive_can_frame" << std::endl;
   
    if (msg) {
        can_msgs::msg::Frame can_frame = *msg;
        // rest of the code
        int msg_node_id = can_frame.id;// Extract node ID (leftmost 7 bits)
        std::cout << "The node id is " << msg_node_id << "\n";
        int operation = can_frame.data[0];

        if (operation == 48){
            std::cout << "Operation is 48 so angle is calculated\n";
            parse_motor_angle(can_frame);
    }

    } else {
        std::cout << "Null pointer in receive_can_frame" << std::endl;
        return;
    }
    // if (msg_node_id != this->node_id) {  // Stop if the message node ID is not the one we want
    //     RCLCPP_DEBUG(this->get_logger(), "Ignoring message for node ID %i", msg_node_id);
    //     return;
    // }

    // switch (msg_pdo) {  // Determine how to parse the message based on PDO

    //     case curtis_constants::MOTOR_INFORMATION_PDO:  // Motor information message
    //         this->parse_motor_info(*msg);
    //         break;

    //     case curtis_constants::MOTOR_MEASUREMENT_PDO:  // Motor measurement message
    //         this->parse_motor_measurement(*msg);
    //         break;

    //     case curtis_constants::MOTOR_COMMAND_PDO:  // Motor command message
    //         this->parse_motor_command(*msg);
    //         break;

    //     case curtis_constants::MOTOR_MISC_PDO:  // Miscellaneous motor information message
    //         this->parse_motor_misc(*msg);
    //         break;

    //     default:
    //         RCLCPP_DEBUG(this->get_logger(), "Ignoring message with PDO 0x%X", msg_pdo);
    // }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Program Start");
    auto node = std::make_shared<servo42_can_decoder>();
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Program End");
    rclcpp::shutdown();
    return 0;
}