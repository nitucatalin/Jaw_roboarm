// =============================================================================
//  xl430_w250_t_driver.hpp
//  Dynamixel XL430-W250-T bus driver (Protocol 2.0), used by MobileBaseHardwareInterface.
//
//  Changes from the previous version:
//   - Absolute position is read from the encoder (Present Position), not integrated from
//     velocity. The arm can start in any pose; the reading is right from the first cycle.
//   - One GroupSyncRead (velocity + position of all servos) and one GroupSyncWrite (goal
//     velocity of all servos) per control cycle, instead of 12 separate round trips.
//     This is the main fix for the "Overrun might occur" warnings.
//   - Return Delay Time set to 0 (factory default is 500 us per reply).
//   - Every communication result is checked and can be reported.
// =============================================================================
#ifndef XL430_W250_T_DRIVER_HPP
#define XL430_W250_T_DRIVER_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

// Control table addresses (XL430-W250 e-manual). EEPROM registers are writable only with torque off.
#define ADDR_RETURN_DELAY_TIME 9      // 1 byte, EEPROM, unit 2 us
#define ADDR_OPERATING_MODE 11        // 1 byte, EEPROM
#define ADDR_TORQUE_ENABLE 64         // 1 byte
#define ADDR_HARDWARE_ERROR_STATUS 70 // 1 byte, read-only
#define ADDR_GOAL_VELOCITY 104        // 4 bytes, signed, unit 0.229 rpm
#define ADDR_PRESENT_VELOCITY 128     // 4 bytes, signed, unit 0.229 rpm
#define ADDR_PRESENT_POSITION 132     // 4 bytes, signed, 4096 ticks per turn
#define LEN_PRESENT_VEL_POS 8         // velocity (128..131) + position (132..135), read in one block

#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3

#define DXL_CENTER_TICKS 2048             // the URDF zero of every joint (set by the Homing Offset)
#define RAD_TO_DXL_POSITION 651.898646904 // 4096 / (2 * pi)
#define RAD_S_TO_DXL_VELOCITY 41.6999850  // (60 / (2 * pi)) / 0.229

class XL430_W250_t_Driver {
public:
  XL430_W250_t_Driver(const std::string & device_name, const std::vector<uint8_t> & ids)
  : device_(device_name), ids_(ids)
  {
    port_ = dynamixel::PortHandler::getPortHandler(device_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_, packet_, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VEL_POS);
    sync_write_velocity_ = std::make_unique<dynamixel::GroupSyncWrite>(port_, packet_, ADDR_GOAL_VELOCITY, 4);
    sync_write_torque_ = std::make_unique<dynamixel::GroupSyncWrite>(port_, packet_, ADDR_TORQUE_ENABLE, 1);
    for (uint8_t id : ids_) sync_read_->addParam(id);
  }

  ~XL430_W250_t_Driver()
  {
    if (is_open_) port_->closePort();
    delete port_;
  }

  // Opens the serial port. Returns false and fills `error` on failure.
  bool open(int baud_rate, std::string & error)
  {
    if (!port_->openPort()) {
      error = "cannot open " + device_ + " (does it exist? is the user in the dialout group? is another program using it?)";
      return false;
    }
    if (!port_->setBaudRate(baud_rate)) {
      error = "cannot set baud rate " + std::to_string(baud_rate) + " on " + device_;
      port_->closePort();
      return false;
    }
    is_open_ = true;
    return true;
  }

  // FTDI latency timer in ms (Linux default 16). Returns -1 if it cannot be read (VM, non-FTDI adapter).
  int latencyTimerMs() const
  {
    const std::string base = device_.substr(device_.find_last_of('/') + 1);
    std::ifstream f("/sys/bus/usb-serial/devices/" + base + "/latency_timer");
    int ms = -1;
    if (!(f >> ms)) return -1;
    return ms;
  }

  bool ping(uint8_t id, uint16_t & model)
  {
    uint8_t err = 0;
    return packet_->ping(port_, id, &model, &err) == COMM_SUCCESS;
  }

  // Torque off, velocity mode and zero return delay, each written only if different
  // (EEPROM has a limited number of write cycles). Returns false and fills `error` on failure.
  bool configureForVelocityControl(uint8_t id, std::string & error)
  {
    if (!write1(id, ADDR_TORQUE_ENABLE, 0, error)) return false;
    if (!write1IfDifferent(id, ADDR_OPERATING_MODE, OPERATING_MODE_VELOCITY, error)) return false;
    if (!write1IfDifferent(id, ADDR_RETURN_DELAY_TIME, 0, error)) return false;
    return write4(id, ADDR_GOAL_VELOCITY, 0, error);
  }

  // Hardware Error Status (register 70). Returns false if it cannot be read.
  bool readHardwareError(uint8_t id, uint8_t & status)
  {
    uint8_t err = 0;
    return packet_->read1ByteTxRx(port_, id, ADDR_HARDWARE_ERROR_STATUS, &status, &err) == COMM_SUCCESS;
  }

  static std::string describeHardwareError(uint8_t s)
  {
    if (s == 0) return "none";
    std::string out;
    auto add = [&out](const char * t) { out += out.empty() ? "" : " + "; out += t; };
    if (s & 0x01) add("Input Voltage");
    if (s & 0x04) add("Overheating");
    if (s & 0x10) add("Electrical Shock");
    if (s & 0x20) add("Overload (persistent load above the limit, usually a mechanical block)");
    return out;
  }

  // Reboot clears a latched hardware error (RAM resets: torque off). EEPROM settings are kept.
  bool reboot(uint8_t id)
  {
    uint8_t err = 0;
    return packet_->reboot(port_, id, &err) == COMM_SUCCESS;
  }

  bool setTorqueAll(bool on, std::string & error)
  {
    sync_write_torque_->clearParam();
    uint8_t value = on ? 1 : 0;
    for (uint8_t id : ids_) sync_write_torque_->addParam(id, &value);
    const int res = sync_write_torque_->txPacket();
    sync_write_torque_->clearParam();
    if (res != COMM_SUCCESS) { error = packet_->getTxRxResult(res); return false; }
    return true;
  }

  // One bus transaction for all servos. Positions in rad relative to the URDF zero (2048 ticks),
  // velocities in rad/s, both in servo direction (the caller applies the joint direction).
  // Returns false (outputs untouched) if the transaction failed or a servo did not answer.
  bool readAll(std::vector<double> & position_rad, std::vector<double> & velocity_rad_s, std::string & error)
  {
    const int res = sync_read_->txRxPacket();
    if (res != COMM_SUCCESS) { error = packet_->getTxRxResult(res); return false; }
    for (size_t i = 0; i < ids_.size(); ++i) {
      if (!sync_read_->isAvailable(ids_[i], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VEL_POS)) {
        error = "no data from servo ID " + std::to_string(ids_[i]);
        return false;
      }
    }
    for (size_t i = 0; i < ids_.size(); ++i) {
      // getData returns the raw 32 bits; the cast to int32_t restores the sign (two's complement)
      const int32_t vel = static_cast<int32_t>(sync_read_->getData(ids_[i], ADDR_PRESENT_VELOCITY, 4));
      const int32_t pos = static_cast<int32_t>(sync_read_->getData(ids_[i], ADDR_PRESENT_POSITION, 4));
      position_rad[i] = static_cast<double>(wrapTicks(pos - DXL_CENTER_TICKS)) / RAD_TO_DXL_POSITION;
      velocity_rad_s[i] = static_cast<double>(vel) / RAD_S_TO_DXL_VELOCITY;
    }
    return true;
  }

  // Fold a reading into [-2048, 2047] ticks (+/-180 deg) around the URDF zero.
  // At power-on a servo reports its 0..4095 encoder count plus the Homing Offset, so with a large
  // offset the 0/4095 jump lands inside the joint's range (joint1 at -20 deg would read +340 deg).
  // No joint of this arm moves more than 180 deg from zero, so the folded value is always the real one.
  static int32_t wrapTicks(int32_t ticks)
  {
    int32_t t = ((ticks % 4096) + 4096) % 4096;
    return t >= 2048 ? t - 4096 : t;
  }

  // True if the last readAll() saw the Hardware Alert bit (bit 7 of the status packet error field).
  bool alertOn(size_t index)
  {
    uint8_t err = 0;
    sync_read_->getError(ids_[index], &err);
    return (err & 0x80) != 0;
  }

  // One bus transaction, no reply from the servos.
  bool writeVelocities(const std::vector<double> & velocity_rad_s, std::string & error)
  {
    sync_write_velocity_->clearParam();
    for (size_t i = 0; i < ids_.size(); ++i) {
      double v = std::isfinite(velocity_rad_s[i]) ? velocity_rad_s[i] : 0.0;
      const int32_t raw = static_cast<int32_t>(std::lround(v * RAD_S_TO_DXL_VELOCITY));
      const uint32_t u = static_cast<uint32_t>(raw);
      uint8_t data[4] = {DXL_LOBYTE(DXL_LOWORD(u)), DXL_HIBYTE(DXL_LOWORD(u)),
                         DXL_LOBYTE(DXL_HIWORD(u)), DXL_HIBYTE(DXL_HIWORD(u))};
      sync_write_velocity_->addParam(ids_[i], data);
    }
    const int res = sync_write_velocity_->txPacket();
    sync_write_velocity_->clearParam();
    if (res != COMM_SUCCESS) { error = packet_->getTxRxResult(res); return false; }
    return true;
  }

  const std::string & device() const { return device_; }

private:
  bool check(int res, uint8_t err, uint8_t id, uint16_t addr, std::string & error)
  {
    if (res != COMM_SUCCESS) {
      error = "ID " + std::to_string(id) + ", register " + std::to_string(addr) + ": " + packet_->getTxRxResult(res);
      return false;
    }
    if ((err & 0x7F) != 0) {
      error = "ID " + std::to_string(id) + ", register " + std::to_string(addr) + ": " + packet_->getRxPacketError(err);
      return false;
    }
    return true;
  }
  bool write1(uint8_t id, uint16_t addr, uint8_t value, std::string & error)
  {
    uint8_t err = 0;
    return check(packet_->write1ByteTxRx(port_, id, addr, value, &err), err, id, addr, error);
  }
  bool write4(uint8_t id, uint16_t addr, uint32_t value, std::string & error)
  {
    uint8_t err = 0;
    return check(packet_->write4ByteTxRx(port_, id, addr, value, &err), err, id, addr, error);
  }
  bool write1IfDifferent(uint8_t id, uint16_t addr, uint8_t value, std::string & error)
  {
    uint8_t current = 0, err = 0;
    if (!check(packet_->read1ByteTxRx(port_, id, addr, &current, &err), err, id, addr, error)) return false;
    return current == value ? true : write1(id, addr, value, error);
  }

  std::string device_;
  std::vector<uint8_t> ids_;
  dynamixel::PortHandler * port_ = nullptr;
  dynamixel::PacketHandler * packet_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_velocity_, sync_write_torque_;
  bool is_open_ = false;
};

#endif  // XL430_W250_T_DRIVER_HPP
