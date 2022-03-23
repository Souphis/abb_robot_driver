#ifndef ABB_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define ABB_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ros2_control hardware_interface
#include <hardware_interface/visibility_control.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ABB
#include <abb_egm_rws_managers/egm_manager.h>
#include <abb_egm_rws_managers/rws_manager.h>

namespace abb {
namespace robot {
class ABBPositionHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ABBPositionHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo &system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) final;

  hardware_interface::return_type read() final;
  hardware_interface::return_type write() final;

 protected:
  struct EGMChannelParameters {
    std::string name;
    unsigned short port_number;
    std::string mech_unit_group;
  };

  bool initialize_rws_manager();

  bool initialize_egm_manager();

  void initialize_egm_parameters();

  bool verify_parameters(const EGMChannelParameters &egm_channel_parameters);

  std::unique_ptr<EGMManager> egm_manager_;
  std::unique_ptr<RWSManager> rws_manager_;

  RobotControllerDescription robot_controller_description_;

  MotionData motion_data_;

  std::vector<EGMChannelParameters> egm_channel_parameters_;

  std::string channel_name_;
  unsigned short port_number_;
  std::string mech_unit_group_;

  std::atomic<bool> egm_read_ok_;
  std::atomic<bool> egm_active_;
  std::atomic<bool> egm_client_running_;
  std::atomic<bool> position_controller_running_;
  std::atomic<bool> velocity_controller_running_;
};
}  // namespace robot
}  // namespace abb

#endif  // ABB_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP
