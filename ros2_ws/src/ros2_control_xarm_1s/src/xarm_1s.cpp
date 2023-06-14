#include "ros2_control_xarm_1s/xarm_1s.hpp"

#include <cassert>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <libserial/SerialPort.h>

namespace ros2_control_xarm_1s
{

hardware_interface::CallbackReturn XArm1SSystemHardware::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  prev_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SSystemHardware::on_configure(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    prev_hw_commands_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm1SSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> XArm1SSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XArm1SSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn XArm1SSystemHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    prev_hw_commands_[i] = hw_commands_[i] = hw_states_[i];
  }

  // TO-DO: turn this into a ROS parameter
  serial_port_.Open("/dev/ttyUSB0");
  serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

  RCLCPP_INFO(rclcpp::get_logger("XArm1SSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm1SSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  serial_port_.Close();

  RCLCPP_INFO(rclcpp::get_logger("XArm1SSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

int XArm1SSystemHardware::servo_id_from_name_(std::string name)
{
  return (name.back() - '1') + 1;
}

std::string XArm1SSystemHardware::servo_name_from_id_(int id)
{
  return "arm" + std::to_string(id);
}

hardware_interface::return_type XArm1SSystemHardware::read(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  char c;
  uint8_t servo_count = hw_states_.size();

  // Prepare message to request servo positions
  LibSerial::DataBuffer msg_out = {
    0x55, 0x55,
    (uint8_t)(servo_count + 3),  // Message Length
    21,                          // Command
    servo_count,                 // Count
  };
  for (auto i = 0; i < servo_count; i++)
    msg_out.push_back(servo_id_from_name_(info_.joints[i].name));
  const int BUFFER_SIZE = 3 * servo_count;
  LibSerial::DataBuffer response(BUFFER_SIZE);

  // Clear any potentially lingering, orphaned messages from the arm
  while (serial_port_.IsDataAvailable())
    serial_port_.ReadByte(c);

  // Send message to the arm
  serial_port_.Write(msg_out);

  // Expect specific header in return
  serial_port_.ReadByte(c);
  if (c != 0x55)
    return hardware_interface::return_type::OK;
  serial_port_.ReadByte(c);
  if (c != 0x55)
    return hardware_interface::return_type::OK;
  serial_port_.ReadByte(c);
  if (c != (uint8_t)(servo_count * 3 + 3))
    return hardware_interface::return_type::OK;
  serial_port_.ReadByte(c);
  if (c != 21)
    return hardware_interface::return_type::OK;
  serial_port_.ReadByte(c);
  if (c != servo_count)
    return hardware_interface::return_type::OK;
  serial_port_.Read(response, BUFFER_SIZE);

  // Read servo positions
  for (auto i = 0; i < servo_count; i++)
  {
    auto servo_name = servo_name_from_id_(response[i*3+0]);
    int servo_index = -1;
    for (int ii = 0; ii < (int)info_.joints.size(); ii++)
    {
      if (info_.joints[ii].name.compare(servo_name) == 0)
      {
        servo_index = ii;
        break;
      }
    }
    if (servo_index == -1) // Should never happen
      continue;

    double units = (response[i*3+2] << 8) + response[i*3+1];
    double radians = 0.0;
    if (servo_name.compare("arm1") == 0)
      radians = ((kGripperUnitsRange - (units - kGripperUnitsMin)) / kGripperUnitsRange) * kGripperUnitsPerRadian;
    else if (servo_name.compare("arm3") == 0 || servo_name.compare("arm5") == 0)
      radians = ((kServoUnitsMax - units) - kServoUnitsMiddle) * kServoUnitsPerRadian;
    else
      radians = (units - kServoUnitsMiddle) * kServoUnitsPerRadian;

    hw_states_[servo_index] = radians;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArm1SSystemHardware::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  uint8_t servo_count = hw_commands_.size();

  // If commands are the same as before, don't send message to arm
  bool has_new_commands = false;
  for (int i = 0; i < servo_count; i++)
    if (hw_commands_[i] != prev_hw_commands_[i])
      has_new_commands = true;
  if (!has_new_commands)
    return hardware_interface::return_type::OK;

  // Prepare message
  LibSerial::DataBuffer msg_out = {
    0x55, 0x55,
    (uint8_t)(servo_count * 3 + 5),  // Message Length
    3,                               // Command
    servo_count,                     // Count
    0xF4,                            // Time (LSB)
    0x01,                            // Time (MSB)
  };
  for (uint8_t servo_index = 0; servo_index < servo_count; servo_index++)
  {
    auto servo_name = info_.joints[servo_index].name;

    double radians = hw_commands_[servo_index];
    double units = 0.0;
    if (servo_name.compare("arm1") == 0)
      units = kGripperUnitsMin + (kGripperUnitsRange - ((radians / kGripperUnitsPerRadian) * kGripperUnitsRange));
    else if (servo_name.compare("arm3") == 0 || servo_name.compare("arm5") == 0)
      units = (kServoUnitsMax - ((radians / kServoUnitsPerRadian) + kServoUnitsMiddle));
    else
      units = (radians / kServoUnitsPerRadian) + kServoUnitsMiddle;

    msg_out.push_back(servo_id_from_name_(servo_name));
    msg_out.push_back(((int)units) & 0x00ff);
    msg_out.push_back((((int)units) & 0xff00) >> 8);
  }

  // Send message to arm
  serial_port_.Write(msg_out);

  // Store commands, to compare against next time
  for (int i = 0; i < servo_count; i++)
    prev_hw_commands_[i] = hw_commands_[i];

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_xarm_1s

PLUGINLIB_EXPORT_CLASS(ros2_control_xarm_1s::XArm1SSystemHardware, hardware_interface::SystemInterface)
