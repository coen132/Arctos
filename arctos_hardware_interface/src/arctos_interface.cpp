#include "arctos_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arctos_interface

{
    hardware_interface::CallbackReturn ArctosInterface::on_init(const hardware_interface::HardwareInfo & info) 
    {
        if (hardware_interface::SystemInterface::on_init(info) !=  hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR; 
        }

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); 
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());    

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size()!= 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("AcrtosInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("AcrtosInterface"),
                "Joint '%s' has %s command interfaces found. %s expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size()!= 2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("AcrtosInterface"),
                "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("AcrtosInterface"),
                "Joint '%s' has %s command interfaces found. %s expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("AcrtosInterface"),
                "Joint '%s' has %s command interfaces found. %s expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn ArctosInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        for (uint i = 0; i< hw_states_position_.size(); i++)
        {
            hw_states_position_[i] = 0;
            hw_states_velocity_[i] = 0;
            hw_commands_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("AcrtosInterface"), "Succesfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArctosInterface::on_activate(const rclcpp_lifecycle::State &)
    {
            return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArctosInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
            return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArctosInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArctosInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type ArctosInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArctosInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_interface::ArctosInterface, hardware_interface::SystemInterface)