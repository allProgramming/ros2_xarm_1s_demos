#ifndef ROS2_CONTROL_XARM_1S__XARM_1S_HPP_
#define ROS2_CONTROL_XARM_1S__XARM_1S_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

namespace ros2_control_xarm_1s
{
class XArm1SSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XArm1SSystemHardware)

  inline static const double kServoUnitsPerRadian = 0.0041887887;
  inline static const double kServoUnitsMax = 1000;
  inline static const double kServoUnitsMin = 0;
  inline static const double kServoUnitsMiddle = (kServoUnitsMax + kServoUnitsMin) / 2;

  inline static const double kGripperUnitsPerRadian = 1.8326;
  inline static const double kGripperUnitsMax = 700;
  inline static const double kGripperUnitsMin = 120;
  inline static const double kGripperUnitsRange = kServoUnitsMax - kServoUnitsMin;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  int servo_id_from_name_(std::string servoName);
  std::string servo_name_from_id_(int servoID);

  LibSerial::SerialPort serial_port_;
  std::vector<double> prev_hw_commands_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace ros2_control_xarm_1s

#endif  // ROS2_CONTROL_XARM_1S__XARM_1S_HPP_
