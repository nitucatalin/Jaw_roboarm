#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/xl430_w250_t_driver.hpp"
#include "rclcpp/macros.hpp"

namespace mobile_base_hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MobileBaseHardwareInterface)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        static constexpr int kMaxConsecutiveReadFailures = 10;  // 0.2 s at 50 Hz

        // Startup calibration: moves the arm through named poses inside on_activate(), before any
        // controller starts. Returns false if the pose was not reached; sets fault_ on a servo error.
        bool move_to(const std::string & name, const std::vector<double> & target);
        void stop_all();

        bool calibrate_on_start_ = true;
        double calibration_velocity_ = 0.5;      // peak joint speed of the startup moves [rad/s]
        std::vector<double> calibration_home_;   // joint1..joint6 [rad]
        std::vector<double> calibration_idle_;   // joint1..joint6 [rad]

        std::unique_ptr<XL430_W250_t_Driver> driver_;
        std::string port_;
        int baud_rate_ = 1000000;
        std::vector<uint8_t> servo_ids_;
        std::vector<double> direction_;            // +1 or -1 per joint (servo sense vs URDF axis)
        std::vector<double> min_pos_, max_pos_;    // URDF position limits, used to stop motion past them
        std::vector<bool> has_limits_;

        std::vector<double> servos_pos_, servos_vel_, servos_cmd_vel_;  // joint frame, exported to ros2_control
        std::vector<double> raw_pos_, raw_vel_, raw_cmd_;                // servo frame, exchanged with the driver
        int consecutive_read_failures_ = 0;
        bool fault_ = false;
    };

}  // namespace mobile_base_hardware

#endif
