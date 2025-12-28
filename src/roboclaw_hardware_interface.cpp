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

#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

#include <iostream>
#include <sstream>
#include <roboclaw_serial/device.hpp>

namespace roboclaw_hardware_interface
{

CallbackReturn RoboClawHardwareInterface::on_init(const HardwareInfo & hardware_info)
{
  // Parse status publish rate (Hz) if provided
  auto rate_param = hardware_info.hardware_parameters.find("status_publish_rate_hz");
  if (rate_param != hardware_info.hardware_parameters.end()) {
    try {
      status_publish_rate_hz_ = std::stod(rate_param->second);
      if (status_publish_rate_hz_ <= 0.0) {
        status_publish_rate_hz_ = 10.0;
      }
    } catch (const std::exception &) {
      std::cerr << "Failed to parse status_publish_rate_hz, defaulting to 10Hz" << std::endl;
      status_publish_rate_hz_ = 10.0;
    }
  }

  // Validate serial port parameter
  std::string serial_port;
  try {
    serial_port = hardware_info.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    std::cerr << "Serial port must be defined as a hardware parameters." << std::endl;
    return CallbackReturn::ERROR;
  }

  // Try to establish a connection to the roboclaw
  try {
    // Read the serial port from hardware parameters
    auto device = std::make_shared<roboclaw_serial::SerialDevice>(serial_port);
    interface_ = std::make_shared<roboclaw_serial::Interface>(device);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return CallbackReturn::FAILURE;
  }

  // Validate parameters describing roboclaw joint configurations
  RoboClawConfiguration config;
  try {
    config = parse_roboclaw_configuration(hardware_info);
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
    return CallbackReturn::ERROR;
  }

  // Initialize each roboclaw unit from validated configuration
  for (auto & [roboclaw_address, joints] : config) {
    roboclaw_units_.push_back(
      RoboClawUnit(interface_, roboclaw_address, joints["M1"], joints["M2"]));
  }

  status_node_ = std::make_shared<rclcpp::Node>("roboclaw_status");
  status_publisher_ = status_node_->create_publisher<msg::MotorControllerState>(
    "roboclaw/status", rclcpp::SystemDefaultsQoS());
  start_status_thread();

  return CallbackReturn::SUCCESS;
}

RoboClawHardwareInterface::~RoboClawHardwareInterface()
{
  stop_status_thread();
}

std::vector<StateInterface> RoboClawHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto roboclaw : roboclaw_units_) {
    for (auto & joint : roboclaw.joints) {
      if (joint) {
        state_interfaces.emplace_back(joint->name, "position", joint->getPositionStatePtr());
      }
    }
  }
  return state_interfaces;
}

std::vector<CommandInterface> RoboClawHardwareInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto roboclaw : roboclaw_units_) {
    for (auto & joint : roboclaw.joints) {
      if (joint) {
        command_interfaces.emplace_back(joint->name, "velocity", joint->getVelocityCommandPtr());
      }
    }
  }
  return command_interfaces;
}

return_type RoboClawHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & roboclaw : roboclaw_units_) {
    roboclaw.write();
  }
  return return_type::OK;
}

return_type RoboClawHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & roboclaw : roboclaw_units_) {
    roboclaw.read();
  }
  return return_type::OK;
}

RoboClawConfiguration RoboClawHardwareInterface::parse_roboclaw_configuration(
  const HardwareInfo & hardware_info)
{
  // Define the configuration map
  RoboClawConfiguration roboclaw_config;

  // Loop over all motors and associate them with their respective roboclaws
  for (auto joint : hardware_info.joints) {
    // We currently only support velocity command interfaces
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "velocity") {
      throw std::runtime_error(
              "Invalid command interface for " + joint.name +
              ". Only velocity command interfaces are supported.");
    }

    // We currently only support position state interfaces
    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "position") {
      throw std::runtime_error(
              "Invalid state interface for " + joint.name +
              ". Only position state interfaces are supported.");
    }

    // Capture and validate parameters
    uint8_t roboclaw_address;
    try {
      roboclaw_address = static_cast<uint8_t>(stoi(joint.parameters.at("address")));

      if (roboclaw_address < 0x80 || roboclaw_address > 0x87) {
        throw std::runtime_error(joint.name + ": Addresses must be in the range [128:136]");
      }
    } catch (const std::invalid_argument & e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(joint.name + ": Address must be an integer.");
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error("Problem looking up address and converting to uint8_t");
    }

    // Get the tick count per wheel rotation value
    int qppr;
    try {
      qppr = stoi(joint.parameters.at("qppr"));
    } catch (const std::out_of_range &) {
      throw std::runtime_error("qppr is not set for " + joint.name);
    } catch (const std::invalid_argument &) {
      throw std::runtime_error("qppr is not numeric for " + joint.name);
    }

    // Get the type of motor from joint parameters
    std::string motor_type;
    try {
      motor_type = joint.parameters.at("motor_type");
    } catch (const std::out_of_range &) {
      throw std::runtime_error(
              "Motor type not set for " + joint.name + ". It must be either M1 or M2.");
    }

    // Ensure that the motor type is valid
    if (motor_type != "M1" && motor_type != "M2") {
      throw std::runtime_error(
              "Motor type for " + joint.name + " must be either M1 or M2 (" + motor_type +
              " provided).");
    }

    // Ensure that a key exists for this address, otherwise initialize default values
    roboclaw_config.emplace(
      roboclaw_address,
      std::map<std::string, MotorJoint::SharedPtr>({{"M1", nullptr}, {"M2", nullptr}}));

    // Ensure that this motor has not already been configured
    if (!roboclaw_config[roboclaw_address][motor_type]) {
      // Set configuration parameters for this motor
      roboclaw_config[roboclaw_address][motor_type] =
        std::make_shared<MotorJoint>(joint.name, qppr);
    } else {
      throw std::runtime_error(
              "Bad motor type " + motor_type + " specified for joint " + joint.name);
    }
  }

  return roboclaw_config;
}

void RoboClawHardwareInterface::publish_status()
{
  if (!status_publisher_ || roboclaw_units_.empty()) {
    return;
  }

  msg::MotorControllerState status_msg;
  std::ostringstream error_stream;

  try {
    // This status message supports a single 2-channel roboclaw; use the first configured unit.
    const auto address = roboclaw_units_.front().get_address();

    roboclaw_serial::VelocityPIDConstantsM1 m1_pid;
    roboclaw_serial::VelocityPIDConstantsM2 m2_pid;
    interface_->read(m1_pid, address);
    interface_->read(m2_pid, address);

    auto [m1_p, m1_i, m1_d, m1_qpps] = m1_pid.fields;
    auto [m2_p, m2_i, m2_d, m2_qpps] = m2_pid.fields;

    status_msg.m1_p = static_cast<float>(m1_p);
    status_msg.m1_i = static_cast<float>(m1_i);
    status_msg.m1_d = static_cast<float>(m1_d);
    status_msg.m1_qpps = static_cast<uint32_t>(m1_qpps);

    status_msg.m2_p = static_cast<float>(m2_p);
    status_msg.m2_i = static_cast<float>(m2_i);
    status_msg.m2_d = static_cast<float>(m2_d);
    status_msg.m2_qpps = static_cast<uint32_t>(m2_qpps);

    roboclaw_serial::EncoderSpeedM1 m1_speed;
    roboclaw_serial::EncoderSpeedM2 m2_speed;
    interface_->read(m1_speed, address);
    interface_->read(m2_speed, address);
    auto [m1_speed_ticks, m1_speed_status] = m1_speed.fields;
    auto [m2_speed_ticks, m2_speed_status] = m2_speed.fields;
    status_msg.m1_current_speed = m1_speed_ticks;
    status_msg.m2_current_speed = m2_speed_ticks;

    roboclaw_serial::MotorCurrents currents;
    interface_->read(currents, address);
    auto [m1_current_raw, m2_current_raw] = currents.fields;
    status_msg.m1_motor_current = static_cast<float>(m1_current_raw) / 100.0F;
    status_msg.m2_motor_current = static_cast<float>(m2_current_raw) / 100.0F;

    roboclaw_serial::EncoderCountM1 m1_encoder;
    roboclaw_serial::EncoderCountM2 m2_encoder;
    interface_->read(m1_encoder, address);
    interface_->read(m2_encoder, address);
    auto [m1_encoder_value, m1_encoder_status] = m1_encoder.fields;
    auto [m2_encoder_value, m2_encoder_status] = m2_encoder.fields;
    status_msg.m1_encoder_value = static_cast<uint32_t>(m1_encoder_value);
    status_msg.m2_encoder_value = static_cast<uint32_t>(m2_encoder_value);
    status_msg.m1_encoder_status = m1_encoder_status;
    status_msg.m2_encoder_status = m2_encoder_status;

    // Speed status bytes are used if encoder status is zero
    if (status_msg.m1_encoder_status == 0U) {
      status_msg.m1_encoder_status = m1_speed_status;
    }
    if (status_msg.m2_encoder_status == 0U) {
      status_msg.m2_encoder_status = m2_speed_status;
    }

    roboclaw_serial::MainBatteryVoltage main_batt;
    interface_->read(main_batt, address);
    status_msg.main_battery_voltage = static_cast<float>(std::get<0>(main_batt.fields)) / 10.0F;

    roboclaw_serial::LogicBatteryVoltage logic_batt;
    interface_->read(logic_batt, address);
    status_msg.logic_battery_voltage = static_cast<float>(std::get<0>(logic_batt.fields)) / 10.0F;

    roboclaw_serial::Temperature temp;
    interface_->read(temp, address);
    status_msg.temperature = static_cast<float>(std::get<0>(temp.fields)) / 10.0F;

    roboclaw_serial::Status status_bits;
    interface_->read(status_bits, address);
    auto status_value = std::get<0>(status_bits.fields);

    if (status_value != 0) {
      std::stringstream ss;
      ss << "status=0x" << std::hex << status_value;
      status_msg.error_string = ss.str();
    }
  } catch (const std::exception & e) {
    error_stream << e.what();
  }

  if (status_msg.error_string.empty()) {
    status_msg.error_string = error_stream.str();
  } else if (!error_stream.str().empty()) {
    status_msg.error_string += "; " + error_stream.str();
  }

  status_publisher_->publish(status_msg);
}

void RoboClawHardwareInterface::start_status_thread()
{
  if (status_thread_running_.load() || !status_publisher_) {
    return;
  }

  status_thread_running_.store(true);
  status_thread_ = std::thread(
    [this]() {
      rclcpp::WallRate rate(status_publish_rate_hz_);
      while (rclcpp::ok() && status_thread_running_.load()) {
        publish_status();
        rate.sleep();
      }
    });
}

void RoboClawHardwareInterface::stop_status_thread()
{
  status_thread_running_.store(false);
  if (status_thread_.joinable()) {
    status_thread_.join();
  }
}

}  // namespace roboclaw_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware_interface::RoboClawHardwareInterface, hardware_interface::SystemInterface);
