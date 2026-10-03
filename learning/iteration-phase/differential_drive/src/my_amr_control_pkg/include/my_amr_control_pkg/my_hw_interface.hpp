#ifndef MY_AMR_CONTROL_PKG__MY_HW_INTERFACE_HPP_
#define MY_AMR_CONTROL_PKG__MY_HW_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


namespace my_amr_control_pkg
{

    class MyAMRHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::vector<double> hw_commands_velocity_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;

        rclcpp::Node::SharedPtr hw_node_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_vel_pub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr telemetry_sub_;

        double current_left_pos_ = 0.0;
        double current_right_pos_ = 0.0;
        double current_left_vel_ = 0.0;
        double current_right_vel_ = 0.0;
    };

} // namespace my_amr_control_pkg

#endif // MY_AMR_CONTROL_PKG__MY_HW_INTERFACE_HPP_