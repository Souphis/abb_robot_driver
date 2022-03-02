#include "abb_robot_driver/utils.hpp"

namespace abb {
namespace robot {
namespace utils {

void verify_robotware_version(const RobotWareVersion& rw_version) {
  if (rw_version.major_number() == 6 && rw_version.minor_number() < 7 && rw_version.patch_number() < 1) {
    auto error_message{"Unsupported RobotWare version (" + rw_version.name() + ", need at least 6.07.01)"};
    // RCLCPP_FATAL(ROS_LOG_VERIFY, error_message);
    throw std::runtime_error{error_message};
  }
}

}  // namespace utils
}  // namespace robot
}  // namespace abb
