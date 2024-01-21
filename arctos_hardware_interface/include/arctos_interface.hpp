#ifndef CUSTOM_HARDWARE_HPP_
#define CUSTOM_HARDWARE_HPP_

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arctos_interface
{
class ArctosInterface : public hardware_interface::SystemInterface {
  public:
      ArctosInterface();
      //RCLCPP_SHARED_PTR_DEFINITIONS(ArctosInterface)

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state ) override;

      HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
      std::vector<double> hw_commands_;
      std::vector<double> hw_states_position_;
      std::vector<double> hw_states_velocity_;
      //void request_motor_angle();
      // ROS2 CAN node related stuff
      // std::shared_ptr<rclcpp::Node> p_node_;
      // std::unique_ptr<std::thread> node_thread_;
      // rclcpp::TimerBase::SharedPtr timer_can_activate_;   
      // rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;  // CAN frame subscriber (from encoders)
      // rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_; 
  };
}

#endif