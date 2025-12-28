// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <roboclaw_serial/interface.hpp>

#include "hardware_interface/system_interface.hpp"
#include "roboclaw_hardware_interface/motor_joint.hpp"
#include "roboclaw_hardware_interface/msg/motor_controller_state.hpp"
#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace roboclaw_hardware_interface
{

/// The roboclaw configuration parameters
typedef std::map<uint8_t, std::map<std::string, MotorJoint::SharedPtr>> RoboClawConfiguration;

class RoboClawHardwareInterface : public hardware_interface::SystemInterface
{
public:
  //////////////////
  // CONSTRUCTORS //
  //////////////////

  /// Default constructor for the hardware interface
  RoboClawHardwareInterface() = default;

  ////////////////////////////////
  // SYSTEM INTERFACE OVERRIDES //
  ////////////////////////////////

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
     * \param[in] hardware_info structure with data from URDF.
     * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
     * \returns CallbackReturn::ERROR if any error happens or data are missing.
     */
  CallbackReturn on_init(const HardwareInfo & hardware_info) override;

  ~RoboClawHardwareInterface();

  /// Exports all state interfaces for this hardware interface.
  /**
     * The state interfaces have to be created and transferred according
     * to the hardware info passed in for the configuration.
     *
     * Note the ownership over the state interfaces is transferred to the caller.
     *
     * \return vector of state interfaces
     */
  std::vector<StateInterface> export_state_interfaces() override;

  /// Exports all command interfaces for this hardware interface.
  /**
     * The command interfaces have to be created and transferred according
     * to the hardware info passed in for the configuration.
     *
     * Note the ownership over the state interfaces is transferred to the caller.
     *
     * \return vector of command interfaces
     */
  std::vector<CommandInterface> export_command_interfaces() override;

  /////////////////////////////////
  //  SYSTEM INTERFACE OVERRIDES //
  /////////////////////////////////

  /// Read the current state values from the actuator.
  /**
     * The data readings from the physical hardware has to be updated
     * and reflected accordingly in the exported state interfaces.
     * That is, the data pointed by the interfaces shall be updated.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Write the current command values to the actuator.
  /**
     * The physical hardware shall be updated with the latest value from
     * the exported command interfaces.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Validate and organize hardware parameters from hardware information.
  /**
     * The hardware info is parsed and validated, checking for parameter
     * consistency and validity. Parameters are organized and stored in the
     * roboclaws_ member variable as a
     * std::map<uint8_t, std::map<std::string, std::optional<MotorConfig>>>,
     * which maps a roboclaw address (uint8_t) to a map of optional motor
     * configurations, the keys for which are strings, either "M1" or "M2"
     *
     * \param[in] hardware_info structure with data from URDF.
     *
     */
  RoboClawConfiguration parse_roboclaw_configuration(const HardwareInfo & hardware_info);

private:
  /// Shared pointer to the RoboClaw driver interface
  roboclaw_serial::Interface::SharedPtr interface_;

  /// Vector of uniqely addressable roboclaw units
  std::vector<RoboClawUnit> roboclaw_units_;

  /// ROS node to publish status information
  rclcpp::Node::SharedPtr status_node_;

  /// Publisher for roboclaw status
  rclcpp::Publisher<msg::MotorControllerState>::SharedPtr status_publisher_;

  /// Diagnostic updater
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

  /// Background thread for publishing diagnostics
  std::thread status_thread_;

  /// Flag to control the diagnostics publisher thread
  std::atomic_bool status_thread_running_{false};

  /// Publish diagnostics/status data to ROS
  void publish_status();

  /// Start publishing diagnostics in the background
  void start_status_thread();

  /// Stop publishing diagnostics in the background
  void stop_status_thread();

  /// Parse hardware parameter for status publish rate
  double status_publish_rate_hz_{10.0};

  /// Cache last status for diagnostics
  std::mutex status_mutex_;
  msg::MotorControllerState last_status_msg_;
  bool has_status_msg_{false};

  /// Fill diagnostic status
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}  // namespace roboclaw_hardware_interface

#endif  // ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE_HPP_
