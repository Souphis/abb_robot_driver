cmake_minimum_required(VERSION 3.5)
project(abb_robot_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abb_egm_rws_managers REQUIRED)

get_target_property(abb_libegm_INCLUDE_DIRS ${abb_libegm_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(abb_librws_INCLUDE_DIRS ${abb_librws_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(abb_egm_rws_managers_INCLUDE_DIRS ${abb_egm_rws_managers_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)

set(INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  abb_libegm
  abb_librws
  abb_egm_rws_managers
)

add_library(
	${PROJECT_NAME}
	SHARED
	src/hardware_interface.cpp
)
target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	include
)

target_link_libraries(
    ${PROJECT_NAME}
    abb_egm_rws_managers::abb_egm_rws_managers
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface
										 hardware_interface_plugin.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY controllers description launch/
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
