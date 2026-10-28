#include "my_robot_hardware/mobile_base_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mobile_base_hardware {

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("MobileBaseHardware");
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// ON INIT: read the parameters from the URDF <ros2_control> block. No hardware access yet.
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;
  port_ = hp.count("serial_port") ? hp.at("serial_port") : "/dev/ttyUSB0";
  baud_rate_ = hp.count("baud_rate") ? std::stoi(hp.at("baud_rate")) : 1000000;

  const size_t n = info_.joints.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];
    const auto & p = joint.parameters;
    // Optional per-joint <param>: id (default 1..6 in URDF order)
    servo_ids_.push_back(static_cast<uint8_t>(p.count("id") ? std::stoi(p.at("id")) : static_cast<int>(i + 1)));
    // Direction: joint3 and joint4 servos are mounted opposite to their URDF axes on this arm,
    // so they are reversed by default. A <param name="direction"> in the xacro (1 or -1) overrides
    // the default; it replaces it, it does not multiply it.
    double dir = (joint.name == "joint3" || joint.name == "joint4") ? -1.0 : 1.0;
    if (p.count("direction")) dir = std::stod(p.at("direction")) < 0 ? -1.0 : 1.0;
    direction_.push_back(dir);

    const auto lim = info_.limits.find(joint.name);
    const bool has = lim != info_.limits.end() && lim->second.has_position_limits;
    has_limits_.push_back(has);
    min_pos_.push_back(has ? lim->second.min_position : -kPi);
    max_pos_.push_back(has ? lim->second.max_position : kPi);
  }

  // Startup calibration, configurable from the <hardware> block of the xacro:
  //   calibrate_on_start   true | false                       (default true)
  //   calibration_velocity peak joint speed, rad/s            (default 0.5)
  //   calibration_home     6 joint values, rad                (default = SRDF "home", gripper open)
  //   calibration_idle     6 joint values, rad                (default = SRDF "idle", all 0)
  if (hp.count("calibrate_on_start")) {
    const std::string v = hp.at("calibrate_on_start");
    calibrate_on_start_ = !(v == "false" || v == "False" || v == "0");
  }
  if (hp.count("calibration_velocity")) calibration_velocity_ = std::stod(hp.at("calibration_velocity"));
  auto parse_pose = [&](const char * key, std::vector<double> fallback, std::vector<double> & out) {
    out = fallback;
    if (!hp.count(key)) return true;
    std::istringstream in(hp.at(key));
    std::vector<double> values;
    for (double x; in >> x;) values.push_back(x);
    if (values.size() != n) {
      RCLCPP_ERROR(kLogger, "%s needs %zu values, got %zu.", key, n, values.size());
      return false;
    }
    out = values;
    return true;
  };
  std::vector<double> home_default(n, 0.0), idle_default(n, 0.0);
  if (n == 6) home_default = {0.0, -1.5707, -1.5707, -1.5707, 0.0, 0.0};
  if (!parse_pose("calibration_home", home_default, calibration_home_) ||
      !parse_pose("calibration_idle", idle_default, calibration_idle_) || calibration_velocity_ <= 0.0) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    for (double target : {calibration_home_[i], calibration_idle_[i]}) {
      if (has_limits_[i] && (target < min_pos_[i] || target > max_pos_[i])) {
        RCLCPP_ERROR(kLogger, "Calibration target %.4f for %s is outside its limits [%.4f, %.4f].", target,
                     info_.joints[i].name.c_str(), min_pos_[i], max_pos_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  servos_pos_.assign(n, 0.0);
  servos_vel_.assign(n, 0.0);
  servos_cmd_vel_.assign(n, 0.0);
  raw_pos_.assign(n, 0.0);
  raw_vel_.assign(n, 0.0);
  raw_cmd_.assign(n, 0.0);

  driver_ = std::make_unique<XL430_W250_t_Driver>(port_, servo_ids_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ON CONFIGURE: open the port and make sure every servo answers.
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  std::string error;
  if (!driver_->open(baud_rate_, error)) {
    RCLCPP_ERROR(kLogger, "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const int latency = driver_->latencyTimerMs();
  if (latency > 1) {
    RCLCPP_WARN(kLogger,
      "USB latency timer is %d ms: every reply waits up to that long, which causes overruns. "
      "Set it to 1 ms (see the udev rule in the README).", latency);
  }

  bool all_ok = true;
  for (uint8_t id : servo_ids_) {
    uint16_t model = 0;
    if (!driver_->ping(id, model)) {
      RCLCPP_ERROR(kLogger, "Servo ID %d does not answer (cable, power, ID or baud rate).", id);
      all_ok = false;
    }
  }
  return all_ok ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &servos_pos_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &servos_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &servos_cmd_vel_[i]);
  }
  return command_interfaces;
}

// ON ACTIVATE: clear old errors, set velocity mode, read the ABSOLUTE position, then torque on.
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  std::string error;
  fault_ = false;
  consecutive_read_failures_ = 0;

  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    const uint8_t id = servo_ids_[i];
    uint8_t hw = 0;
    if (driver_->readHardwareError(id, hw) && hw != 0) {
      RCLCPP_WARN(kLogger, "Servo ID %d had a latched error: %s. Rebooting it.", id,
                  XL430_W250_t_Driver::describeHardwareError(hw).c_str());
      driver_->reboot(id);
      std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }
    if (!driver_->configureForVelocityControl(id, error)) {
      RCLCPP_ERROR(kLogger, "Configuring servo: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!driver_->readAll(raw_pos_, raw_vel_, error)) {
    RCLCPP_ERROR(kLogger, "First read failed: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Startup report: where the arm is right now, in the URDF frame.
  std::ostringstream report;
  report << "Arm position at start (URDF zero = 2048 ticks):";
  bool outside = false;
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    servos_pos_[i] = direction_[i] * raw_pos_[i];
    servos_vel_[i] = 0.0;
    servos_cmd_vel_[i] = 0.0;
    const bool out = has_limits_[i] && (servos_pos_[i] < min_pos_[i] - 0.05 || servos_pos_[i] > max_pos_[i] + 0.05);
    outside = outside || out;
    report << "\n  " << info_.joints[i].name << " (ID " << int(servo_ids_[i])
           << (direction_[i] < 0 ? ", reversed" : "") << "): " << std::fixed
           << std::setprecision(3) << servos_pos_[i] << " rad = " << std::setprecision(1)
           << servos_pos_[i] * 180.0 / kPi << " deg" << (out ? "   <-- OUTSIDE THE URDF LIMITS" : "");
  }
  RCLCPP_INFO(kLogger, "%s", report.str().c_str());
  if (outside && !calibrate_on_start_) {
    RCLCPP_ERROR(kLogger,
      "A joint is outside its URDF limits and calibrate_on_start is false, so nothing would bring it back. "
      "Move it into range by hand, or enable the calibration. Torque stays off.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (outside) {
    RCLCPP_WARN(kLogger, "A joint starts outside its URDF limits; the calibration will bring it into range.");
  }

  if (!driver_->setTorqueAll(true, error)) {
    RCLCPP_ERROR(kLogger, "Torque on failed: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Startup calibration: home, then idle. The controller manager waits here; controllers and
  // MoveIt start only after this returns, so they begin from the idle pose.
  if (!calibrate_on_start_) {
    RCLCPP_INFO(kLogger, "Calibration skipped (calibrate_on_start = false).");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(kLogger, "Calibration started: home, then idle (peak speed %.2f rad/s).", calibration_velocity_);
  const bool ok = move_to("home", calibration_home_) && move_to("idle", calibration_idle_);
  if (fault_) {
    RCLCPP_ERROR(kLogger, "Calibration aborted by a servo error. Torque is off on the faulted servo.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!ok) {
    RCLCPP_WARN(kLogger, "Calibration incomplete. The arm holds where it stopped; controllers start from there.");
  } else {
    RCLCPP_INFO(kLogger, "Calibration complete. Controllers start now.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MobileBaseHardwareInterface::stop_all() {
  std::string error;
  std::fill(raw_cmd_.begin(), raw_cmd_.end(), 0.0);
  driver_->writeVelocities(raw_cmd_, error);
}

// One startup move: a smooth (minimum-jerk) joint-space path from the current pose to `target`,
// all joints arriving together, followed with a P correction on top of the path velocity.
// Runs its own 50 Hz loop; the controller manager's loop skips this hardware while it activates.
bool MobileBaseHardwareInterface::move_to(const std::string & name, const std::vector<double> & target) {
  using clock = std::chrono::steady_clock;
  constexpr double kKp = 4.0;          // 1/s: position error -> velocity correction
  constexpr double kVelocityCap = 1.0;  // rad/s: hard cap on any command
  constexpr double kMaxLag = 0.3;       // rad: a joint this far behind its path is blocked -> stop at once
  constexpr double kTolerance = 0.02;   // rad: "reached" (about 1.1 degrees)
  constexpr int kSettledCycles = 5;     // within tolerance for 0.1 s
  const auto period = std::chrono::milliseconds(20);

  RCLCPP_INFO(kLogger, "Calibration: waiting for %s ...", name.c_str());
  std::string error;
  if (!driver_->readAll(raw_pos_, raw_vel_, error)) {
    RCLCPP_ERROR(kLogger, "Calibration: read failed before %s: %s", name.c_str(), error.c_str());
    return false;
  }
  const size_t n = servo_ids_.size();
  std::vector<double> start(n), delta(n);
  double largest = 0.0;
  for (size_t i = 0; i < n; ++i) {
    start[i] = direction_[i] * raw_pos_[i];
    delta[i] = target[i] - start[i];
    largest = std::max(largest, std::fabs(delta[i]));
  }
  // minimum-jerk peak speed = 1.875 * distance / duration
  const double duration = std::max(1.0, 1.875 * largest / calibration_velocity_);
  const double timeout = duration + 3.0;

  const auto t0 = clock::now();
  auto next = t0;
  int settled = 0, read_failures = 0;
  double worst_error = 0.0;
  for (;;) {
    next += period;
    const double t = std::chrono::duration<double>(clock::now() - t0).count();

    if (!driver_->readAll(raw_pos_, raw_vel_, error)) {
      if (++read_failures >= kMaxConsecutiveReadFailures) {
        stop_all();
        RCLCPP_ERROR(kLogger, "Calibration: %d reads in a row failed during %s (%s).", read_failures,
                     name.c_str(), error.c_str());
        return false;
      }
      std::this_thread::sleep_until(next);
      continue;
    }
    read_failures = 0;

    for (size_t i = 0; i < n; ++i) {
      if (driver_->alertOn(i)) {
        stop_all();
        uint8_t hw = 0;
        driver_->readHardwareError(servo_ids_[i], hw);
        RCLCPP_ERROR(kLogger, "Calibration: servo ID %d (%s) stopped itself during %s: %s", servo_ids_[i],
                     info_.joints[i].name.c_str(), name.c_str(), XL430_W250_t_Driver::describeHardwareError(hw).c_str());
        fault_ = true;
        return false;
      }
    }

    const double tau = std::min(t / duration, 1.0);
    const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
    const double ds = tau < 1.0 ? 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / duration : 0.0;
    worst_error = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double q = direction_[i] * raw_pos_[i];
      servos_pos_[i] = q;
      const double lag = start[i] + delta[i] * s - q;
      if (std::fabs(lag) > kMaxLag) {
        stop_all();
        RCLCPP_ERROR(kLogger, "Calibration: %s stopped. %s is %.2f rad behind its path at %.3f rad: blocked by "
                     "an obstacle, or wrong servo direction/homing.", name.c_str(), info_.joints[i].name.c_str(),
                     std::fabs(lag), q);
        return false;
      }
      const double v = std::clamp(delta[i] * ds + kKp * lag, -kVelocityCap, kVelocityCap);
      raw_cmd_[i] = direction_[i] * v;
      worst_error = std::max(worst_error, std::fabs(target[i] - q));
    }
    if (!driver_->writeVelocities(raw_cmd_, error)) {
      RCLCPP_WARN(kLogger, "Calibration: velocity write failed: %s", error.c_str());
    }

    if (tau >= 1.0) {
      settled = worst_error < kTolerance ? settled + 1 : 0;
      if (settled >= kSettledCycles) break;
    }
    if (t > timeout) {
      stop_all();
      std::ostringstream which;
      for (size_t i = 0; i < n; ++i) {
        const double q = direction_[i] * raw_pos_[i];
        if (std::fabs(target[i] - q) >= kTolerance) {
          which << " " << info_.joints[i].name << " at " << std::fixed << std::setprecision(3) << q
                << " (target " << target[i] << ")";
        }
      }
      RCLCPP_ERROR(kLogger, "Calibration: %s not reached after %.1f s:%s. Blocked by an obstacle, or wrong servo "
                   "direction/homing?", name.c_str(), t, which.str().c_str());
      return false;
    }
    std::this_thread::sleep_until(next);
  }

  stop_all();
  const double took = std::chrono::duration<double>(clock::now() - t0).count();
  RCLCPP_INFO(kLogger, "Calibration: %s done (%.1f s, largest error %.3f rad).", name.c_str(), took, worst_error);
  return true;
}

// ON DEACTIVATE: stop, then release torque. Joints 2 and 3 fall under gravity: support the arm.
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  std::string error;
  std::fill(raw_cmd_.begin(), raw_cmd_.end(), 0.0);
  driver_->writeVelocities(raw_cmd_, error);
  driver_->setTorqueAll(false, error);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// READ: one bus transaction for all six servos.
hardware_interface::return_type
MobileBaseHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (fault_) return hardware_interface::return_type::ERROR;

  std::string error;
  if (!driver_->readAll(raw_pos_, raw_vel_, error)) {
    if (++consecutive_read_failures_ >= kMaxConsecutiveReadFailures) {
      RCLCPP_ERROR(kLogger, "%d reads in a row failed (%s).", consecutive_read_failures_, error.c_str());
      for (uint8_t id : servo_ids_) {
        uint8_t hw = 0;
        if (!driver_->readHardwareError(id, hw)) RCLCPP_ERROR(kLogger, "  servo ID %d: no answer", id);
        else if (hw != 0) RCLCPP_ERROR(kLogger, "  servo ID %d: %s", id, XL430_W250_t_Driver::describeHardwareError(hw).c_str());
      }
      fault_ = true;
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;  // keep the last good values for this cycle
  }
  consecutive_read_failures_ = 0;

  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    servos_pos_[i] = direction_[i] * raw_pos_[i];
    servos_vel_[i] = direction_[i] * raw_vel_[i];
    if (driver_->alertOn(i)) {
      uint8_t hw = 0;
      driver_->readHardwareError(servo_ids_[i], hw);
      RCLCPP_ERROR(kLogger, "Servo ID %d (%s) stopped itself: %s", servo_ids_[i], info_.joints[i].name.c_str(),
                   XL430_W250_t_Driver::describeHardwareError(hw).c_str());
      fault_ = true;
    }
  }
  return fault_ ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

// WRITE: one bus transaction, no reply. A command that would push a joint further past its
// URDF limit is set to 0, so a servo never drives itself into a mechanical stop.
hardware_interface::return_type
MobileBaseHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (fault_) return hardware_interface::return_type::ERROR;

  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    double v = std::isfinite(servos_cmd_vel_[i]) ? servos_cmd_vel_[i] : 0.0;
    if (has_limits_[i] && ((servos_pos_[i] >= max_pos_[i] && v > 0.0) || (servos_pos_[i] <= min_pos_[i] && v < 0.0))) {
      v = 0.0;
    }
    raw_cmd_[i] = direction_[i] * v;
  }

  std::string error;
  if (!driver_->writeVelocities(raw_cmd_, error)) {
    RCLCPP_WARN(kLogger, "Velocity write failed: %s", error.c_str());
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mobile_base_hardware

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
