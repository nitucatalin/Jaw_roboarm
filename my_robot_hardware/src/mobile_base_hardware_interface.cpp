#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mobile_base_hardware {

    //ON INIT
    hardware_interface::CallbackReturn
    MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
 
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;
        servo_ids_ = {1, 2, 3, 4, 5, 6};
        
        servos_pos_.assign(6, 0.0);
        servos_vel_.assign(6, 0.0);
        servos_cmd_vel_.assign(6, 0.0);

        port_ = "/dev/ttyUSB0";
        driver_ = std::make_shared<XL430_W250_t_Driver>(port_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ON CONFIGURE
    hardware_interface::CallbackReturn
    MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        if(driver_->init() != 0) return hardware_interface::CallbackReturn::ERROR;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &servos_pos_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &servos_vel_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &servos_cmd_vel_[i]));
        }
        return command_interfaces;
    }

    // ON ACTIVATE
    hardware_interface::CallbackReturn
    MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        for(size_t i = 0; i < servo_ids_.size(); i++) {
            driver_->activateWithVelocityMode(servo_ids_[i]);
            servos_pos_[i] = 0.0;
            servos_vel_[i] = 0.0;
            servos_cmd_vel_[i] = 0.0;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ON DEACTIVATE
    hardware_interface::CallbackReturn
    MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        for(size_t i = 0; i < servo_ids_.size(); i++) {
            driver_->deactivate(servo_ids_[i]);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // READ
    hardware_interface::return_type
    MobileBaseHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        double dt = period.seconds();

        for(size_t i = 0; i < servo_ids_.size(); i++) {
            try {
                servos_vel_[i] = driver_->getVelocityRadianPerSec(servo_ids_[i]);
                servos_pos_[i] += (servos_vel_[i] * dt);
            } catch(...) {
                return hardware_interface::return_type::OK;
            }
        }

        return hardware_interface::return_type::OK;
    }

    // WRITE
    hardware_interface::return_type
    MobileBaseHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        (void)period;

        for(size_t i = 0; i < servo_ids_.size(); i++) {
            try {
                driver_->setTargetVelocityRadianPerSec(servo_ids_[i], servos_cmd_vel_[i]);
            } catch(...) {
                return hardware_interface::return_type::OK;
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)