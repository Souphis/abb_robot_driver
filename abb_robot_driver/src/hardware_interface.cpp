#include "abb_robot_driver/hardware_interface.hpp"

#include <google/protobuf/text_format.h>

#include <algorithm>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "abb_robot_driver/utils.hpp"

namespace abb {
namespace robot {
CallbackReturn ABBPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  info_ = system_info;

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("ABBPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
    }
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  initialize_robot_controller_description();
  try {
    initializeMotionData(motion_data_, robot_controller_description_);
  } catch (...) {
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("egm_mech_unit_group") == info_.hardware_parameters.end()) {
    mech_unit_group_ = "";
  } else {
    mech_unit_group_ = info_.hardware_parameters["egm_mech_unit_group"];
  }
  channel_name_ = info_.hardware_parameters["egm_channel"];
  port_number_ = std::stoi(info_.hardware_parameters["egm_port_number"]);

  std::vector<EGMManager::ChannelConfiguration> egm_channel_configurations{};

  for (const auto& parameters : egm_channel_parameters_) {
    if (verify_parameters(parameters)) {
      auto mug{findMechanicalUnitGroup(parameters.mech_unit_group, robot_controller_description_)};
      egm_channel_configurations.emplace_back(parameters.port_number, mug);
    }
  }

  if (egm_channel_configurations.empty()) {
    return CallbackReturn::ERROR;
  }

  egm_manager_ = std::make_unique<EGMManager>(egm_channel_configurations);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  egm_manager_.reset();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ABBPositionHardwareInterface::read() {
  if (!egm_manager_) throw std::runtime_error{"The EGM manager has not been initialized"};

  egm_read_ok_ = egm_manager_->read(motion_data_);

  for (auto& group : motion_data_.groups) {
    egm_active_ = group.egm_channel_data.is_active;
    egm_client_running_ = group.egm_channel_data.input.status().egm_state() == egm::wrapper::Status::EGM_RUNNING;
    for (auto& unit : group.units) {
      for (auto& joint : unit.joints) {
        joint.command.position = joint.state.position;
        joint.command.velocity = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ABBPositionHardwareInterface::write() {
  if (!egm_manager_) throw std::runtime_error{"The EGM manager has not been initialized"};
  if (egm_read_ok_ && egm_active_ && egm_client_running_) {
    egm_manager_->write(motion_data_);
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ABBPositionHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups) {
    for (auto& unit : group.units) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &unit.joints[i].state.position));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &unit.joints[i].state.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &unit.joints[i].state.effort));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBPositionHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups) {
    for (auto& unit : group.units) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &unit.joints[i].command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &unit.joints[i].command.velocity));
      }
    }
  }
  return command_interfaces;
}

bool ABBPositionHardwareInterface::initialize_robot_controller_description() {
  std::string robot_controller_description = info_.hardware_parameters["robot_controller_description"];

  if (!google::protobuf::TextFormat::ParseFromString(robot_controller_description, &robot_controller_description_)) {
    return false;
  }

  try {
    utils::verify_robotware_version(robot_controller_description_.header().robot_ware_version());
  } catch (...) {
    return false;
  }

  if (!robot_controller_description_.system_indicators().options().egm()) {
    return false;
  }

  return true;
}

bool ABBPositionHardwareInterface::verify_parameters(const EGMChannelParameters& parameters) {
  const auto& options = robot_controller_description_.system_indicators().options();

  auto& groups = motion_data_.groups;
  const auto& mug_name = parameters.mech_unit_group;

  if (mug_name.empty() && options.multimove()) {
    return false;
  }

  auto it{groups.begin()};

  if (options.multimove()) {
    it = std::find_if(groups.begin(), groups.end(), [&](const auto& x) { return x.name == mug_name; });
  }

  if (it == groups.end()) {
    return false;
  }
  auto count =
      std::count_if(it->units.begin(), it->units.end(), [](const auto& unit) { return unit.supported_by_egm; });

  if (count == 0) {
    return false;
  }

  it->egm_channel_data.name = parameters.name;
  it->egm_channel_data.port_number = parameters.port_number;

  return true;
}
}  // namespace robot
}  // namespace abb

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb::robot::ABBPositionHardwareInterface, hardware_interface::SystemInterface)
