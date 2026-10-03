#include "my_amr_control_pkg/my_hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_amr_control_pkg
{

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Do wheels ke liye memory allocate karein: [0] = Left wheel, [1] = Right wheel
        hw_commands_velocity_.resize(info_.joints.size(), 0.0);
        hw_states_position_.resize(info_.joints.size(), 0.0);
        hw_states_velocity_.resize(info_.joints.size(), 0.0);

        // ESP32 micro-ROS ya external system se communicate karne ke liye internal ROS 2 node banayein
        hw_node_ = std::make_shared<rclcpp::Node>("amr_hw_bridge_node");

        // Left aur Right wheel velocities ke liye separate publishers banayein (in rad/s)
        left_vel_pub_ = hw_node_->create_publisher<std_msgs::msg::Float64>("/left_vel", 10);
        right_vel_pub_ = hw_node_->create_publisher<std_msgs::msg::Float64>("/right_vel", 10);

        // Subscribe to encoder telemetry
        telemetry_sub_ = hw_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/encoder_telemetry", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // Loop through the names to find left and right wheels dynamically
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == "left_wheel") {
                        if (msg->position.size() > i) current_left_pos_ = msg->position[i];
                        if (msg->velocity.size() > i) current_left_vel_ = msg->velocity[i];
                    } else if (msg->name[i] == "right_wheel") {
                        if (msg->position.size() > i) current_right_pos_ = msg->position[i];
                        if (msg->velocity.size() > i) current_right_vel_ = msg->velocity[i];
                    }
                }
            });



        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MyAMRHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyAMRHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (size_t i = 0; i < hw_commands_velocity_.size(); ++i)
        {
            hw_commands_velocity_[i] = 0.0;
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MyAMRHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Process incoming messages (updates positions and velocities)
        rclcpp::spin_some(hw_node_);

        // Update system memory directly from hardware feedback
        hw_states_position_[0] = current_left_pos_;
        hw_states_velocity_[0] = current_left_vel_;

        hw_states_position_[1] = current_right_pos_;
        hw_states_velocity_[1] = current_right_vel_;

        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type MyAMRHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // diff_drive_controller se aayi wheel velocities ko publish karein (in rad/s)
        std_msgs::msg::Float64 left_msg;
        left_msg.data = hw_commands_velocity_[0];
        left_vel_pub_->publish(left_msg);

        std_msgs::msg::Float64 right_msg;
        right_msg.data = hw_commands_velocity_[1];
        right_vel_pub_->publish(right_msg);

        return hardware_interface::return_type::OK;
    }

} // namespace my_amr_control_pkg

// Yeh macro namespace block ke bahar hona zaroori hai
PLUGINLIB_EXPORT_CLASS(
    my_amr_control_pkg::MyAMRHardwareInterface,
    hardware_interface::SystemInterface)