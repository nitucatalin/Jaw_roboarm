#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/xl430_w250_t_driver.hpp"
#include "rclcpp/macros.hpp"
#include <vector>
#include <string>

namespace mobile_base_hardware {

    class MobileBaseHardwareInterface : public hardware_interface::SystemInterface {

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(MobileBaseHardwareInterface)

            // ON CONFIGURE
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;
            // ON ACTIVATE
            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;
            // ON DEACTIVATE
            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            // ON INIT
            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;
            
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            // READ
            hardware_interface::return_type
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            // WRITE
            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<XL430_W250_t_Driver> driver_;
            std::string port_;
            std::vector<int> servo_ids_;

            std::vector<double> servos_pos_;
            std::vector<double> servos_vel_;
            std::vector<double> servos_cmd_vel_;    

    }; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware

#endif