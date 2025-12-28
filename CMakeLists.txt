cmake_minimum_required(VERSION 3.8)
project(roboclaw_hardware_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roboclaw_serial REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface roboclaw_hardware_interface.xml)

# Include directories
include_directories(
  include
)

# Add roboclaw hardware interface shared library (target name unique to avoid clash with rosidl)
set(ROBOCLAW_HW_LIB_TARGET ${PROJECT_NAME}_core)
add_library(${ROBOCLAW_HW_LIB_TARGET} SHARED
  src/roboclaw_hardware_interface.cpp
  src/roboclaw_unit.cpp
  src/motor_joint.cpp
)
set_target_properties(${ROBOCLAW_HW_LIB_TARGET} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_link_libraries(${ROBOCLAW_HW_LIB_TARGET} roboclaw_serial)

# Add library dependencies
ament_target_dependencies(${ROBOCLAW_HW_LIB_TARGET}
  pluginlib hardware_interface rclcpp
  roboclaw_serial
)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorControllerState.msg"
)

rosidl_get_typesupport_target(
  cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${ROBOCLAW_HW_LIB_TARGET} ${cpp_typesupport_target})

# Install the shared library
install(TARGETS ${ROBOCLAW_HW_LIB_TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  roboclaw_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
