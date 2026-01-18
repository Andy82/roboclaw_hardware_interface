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

#pragma once

#include <cstdint>
#include <string>
#include <tuple>

namespace roboclaw_serial
{

enum class Command : unsigned char
{
  DRIVE_M1_FWD = 0,                   // Drive Forward Motor 1
  DRIVE_M1_BWD = 1,                   // Drive Backwards Motor 1
  SET_MAIN_VOLT_MIN = 2,              // Set Main Voltage Minimum
  SET_MAIN_VOLT_MAX = 3,              // Set Main Voltage Maximum
  DRIVE_M2_FWD = 4,                   // Drive Forward Motor 2
  DRIVE_M2_BWD = 5,                   // Drive Backwards Motor 2
  DRIVE_M1_7BIT = 6,                  // Drive Motor 1 (7 Bit)
  DRIVE_M2_7BIT = 7,                  // Drive Motor 2 (7 Bit)
  DRIVE_FWD_MIXED = 8,                // Drive Forward Mixed Mode
  DRIVE_BWD_MIXED = 9,                // Drive Backwards Mixed Mode
  TURN_RIGHT_MIXED = 10,              // Turn Right Mixed Mode
  TURN_LEFT_MIXED = 11,               // Turn Left Mixed Mode
  DRIVE_7BIT = 12,                    // Drive Forward or Backward (7 bit)
  TURN_7BIT = 13,                     // Turn Left or Right (7 Bit)
  SET_SERIAL_TIMEOUT = 14,            // Set Serial Timeout
  READ_SERIAL_TIMEOUT = 15,           // Read Serial Timeout
  READ_M1_ENC = 16,                   // Read Encoder Count/Value for M1
  READ_M2_ENC = 17,                   // Read Encoder Count/Value for M2
  READ_M1_ENC_SPD = 18,               // Read M1 Speed in Encoder Counts Per Second
  READ_M2_ENC_SPD = 19,               // Read M2 Speed in Encoder Counts Per Second
  RESET_M1_M2_ENC_REG = 20,           // Resets Encoder Registers for M1 and M2 (Quadrature only)
  READ_FIRMWARE_VER = 21,             // Read Firmware Version
  SET_ENC1_REG = 22,                  // Set Encoder 1 Register (Quadrature only)
  SET_ENC2_REG = 23,                  // Set Encoder 2 Register (Quadrature only)
  READ_MAIN_BATT_VOLT = 24,           // Read Main Battery Voltage
  READ_LOGIC_BATT_VOLT = 25,          // Read Logic Battery Voltage
  SET_MIN_LOGIC_VOLT = 26,            // Set Minimum Logic Voltage Level
  SET_MAX_LOGIC_VOLT = 27,            // Set Maximum Logic Voltage Level
  SET_M1_VEL_PID_CONST = 28,          // Set Velocity PID Constants for M1
  SET_M2_VEL_PID_CONST = 29,          // Set Velocity PID Constants for M2
  READ_M1_RAW_SPD = 30,               // Read Current M1 Raw Speed
  READ_M2_RAW_SPD = 31,               // Read Current M2 Raw Speed
  DRIVE_M1_SGN_DUTY = 32,             // Drive M1 With Signed Duty Cycle (Encoders not required)
  DRIVE_M2_SGN_DUTY_CYCLE = 33,       // Drive M2 with Signed Duty Cycle (Encoders not required)
  DRIVE_M1_M2_SGN_DUTY_CYCLE = 34,    // Drive M1/M2 with Signed Duty Cycle (Encoders not required)
  DRIVE_M1_SGN_SPD = 35,              // Drive M1 with Signed Speed
  DRIVE_M2_SGN_SPD = 36,              // Drive M2 with Signed Speed
  DRIVE_M1_M2_SGN_SPD = 37,           // Drive M1/M2 with Signed Speed
  DRIVE_M1_SGN_SPD_ACCEL = 38,        // Drive M1 with Signed Speed and Acceleration
  DRIVE_M2_SGN_SPD_ACCEL = 39,        // Drive M2 with Signed Speed and Acceleration
  DRIVE_M1_M2_SGN_SPD_ACCEL = 40,     // Drive M1/M2 with Signed Speed and Acceleration
  DRIVE_M1_SGN_SPD_DIST_BUF = 41,     // Drive M1 with Signed Speed and Distance (Buffered)
  DRIVE_M2_SGN_SPD_DIST_BUF = 42,     // Drive M2 with Signed Speed and Distance (Buffered)
  DRIVE_M1_M2_SGN_SPD_DIST_BUF = 43,  // Drive M1/M2 with Signed Speed and Distance (Buffered)
  DRIVE_M1_SGN_SPD_ACCEL_DIST_BUF =
    44,  // Drive M1 with Signed Speed, Acceleration and Distance (Buffered)
  DRIVE_M2_SGN_SPD_ACCEL_DIST_BUF =
    45,  // Drive M2 with Signed Speed, Acceleration and Distance (Buffered)
  DRIVE_M1_M2_SGN_SPD_ACCEL_DIST_BUF =
    46,                     // Drive M1/M2 with Signed Speed, Acceleration and Distance (Buffered)
  READ_BUF_LEN = 47,        // Read Buffer Length
  READ_M1_M2_PWM = 48,      // Read Motor PWMs
  READ_M1_M2_CURRENT = 49,  // Read Motor Currents
  DRIVE_M1_M2_SGN_SPD_ACCEL_IND = 50,  // Drive M1/M2 with Individual Signed Speed and Acceleration
  DRIVE_M1_M2_SGN_SPD_ACCEL_DIST_IND =
    51,                          // Drive M1/M2 with Individual Signed Speed, Accel and Distance
  DRIVE_M1_SGN_DUTY_ACCEL = 52,  // Drive M1 with Signed Duty and Accel (Encoders not required)
  DRIVE_M2_SGN_DUTY_ACCEL = 53,  // Drive M2 with Signed Duty and Accel (Encoders not required)
  DRIVE_M1_M2_SGN_DUTY_ACCEL =
    54,                           // Drive M1/M2 with Signed Duty and Accel (Encoders not required)
  READ_M1_VEL_PID_CONST = 55,     // Read Motor 1 Velocity PID Constants
  READ_M2_VEL_PID_CONST = 56,     // Read Motor 2 Velocity PID Constants
  SET_MAIN_BATT_VOLT = 57,        // Set Main Battery Voltages
  SET_LOGIC_BATT_VOLT = 58,       // Set Logic Battery Voltages
  READ_MAIN_BATT_VOLT_SET = 59,   // Read Main Battery Voltage Settings
  READ_LOGIC_BATT_VOLT_SET = 60,  // Read Logic Battery Voltage Settings
  SET_POS_PID_CONST_M1 = 61,      // Set Position PID Constants for M1
  SET_POS_PID_CONST_M2 = 62,      // Set Position PID Constants for M2
  READ_M1_POS_PID_CONST = 63,     // Read Motor 1 Position PID Constants
  READ_M2_POS_PID_CONST = 64,     // Read Motor 2 Position PID Constants
  DRIVE_M1_SPD_ACCEL_DECEL_POS = 65,     // Drive M1 with Speed, Accel, Deccel and Position
  DRIVE_M2_SPD_ACCEL_DECEL_POS = 66,     // Drive M2 with Speed, Accel, Deccel and Position
  DRIVE_M1_M2_SPD_ACCEL_DECEL_POS = 67,  // Drive M1/M2 with Speed, Accel, Deccel and Position
  SET_M1_DUTY_ACCEL = 68,                // Set default duty cycle acceleration for M1
  SET_M2_DUTY_ACCEL = 69,                // Set default duty cycle acceleration for M2
  SET_M1_DEF_SPEED = 70,                 // Set Default Speed for M1
  SET_M2_DEF_SPEED = 71,                 // Set Default Speed for M2
  READ_DEF_SPEED_SET = 72,               // Read Default Speed Settings
  SET_S3S4S5_MODES = 74,                 // Set S3,S4 and S5 Modes
  READ_S3S4S5_MODES = 75,                // Read S3,S4 and S5 Modes
  SET_DEADBAND_RC = 76,                  // Set DeadBand for RC/Analog controls
  READ_DEADBAND_RC = 77,                 // Read DeadBand for RC/Analog controls
  READ_M1_M2_ENC = 78,                   // Read Encoder Counts
  READ_M1_M2_RAW_SPD = 79,               // Read Raw Motor Speeds
  RESTORE_DEFAULTS = 80,                 // Restore Defaults
  READ_DUTY_ACCELS = 81,                 // Read Default Duty Cycle Accelerations
  READ_TEMP = 82,                        // Read Temperature
  READ_TEMP_2 = 83,                      // Read Temperature 2

  READ_STATUS = 90,          // Read Status
  READ_ENCODER_MODES = 91,   // Read Encoder Modes
  SET_M1_ENCODER_MODE = 92,  // Set Motor 1 Encoder Mode
  SET_M2_ENCODER_MODE = 93,  // Set Motor 2 Encoder Mode
  WRITE_SET_EEPROM = 94,     // Write Settings to EEPROM
  READ_SET_EEPROM = 95,      // Read Settings from EEPROM

  SET_STD_CONFIG_SET = 98,       // Set Standard Config Settings
  READ_STD_CONFIG_SET = 99,      // Read Standard Config Settings
  SET_CTRL_MODES = 100,          // Set CTRL Modes
  READ_CTRL_MODES = 101,         // Read CTRL Modes
  SET_CTRL1 = 102,               // Set CTRL1
  SET_CTRL2 = 103,               // Set CTRL2
  READ_CTRLS = 104,              // Read CTRLs
  SET_M1_AUTO_HOME = 105,        // Set Auto Home Duty/Speed and Timeout M1
  SET_M2_AUTO_HOME = 106,        // Set Auto Home Duty/Speed and Timeout M2
  READ_AUTO_HOME_SET = 107,      // Read Auto Home Settings
  READ_M1_M2_AVG_SPD = 108,      // Read Motor Average Speeds
  SET_SPEED_ERR_LIMITS = 109,    // Set Speed Error Limits
  READ_SPEED_ERR_LIMITS = 110,   // Read Speed Error Limits
  READ_SPD_ERR = 111,            // Read Speed Errors
  SET_POS_ERR_LIMITS = 112,      // Set Position Error Limits
  READ_POS_ERR_LIMITS = 113,     // Read Position Error Limits
  READ_POS_ERR = 114,            // Read Position Errors
  SET_BATT_VOLT_OFFSETS = 115,   // Set Battery Voltage Offsets
  READ_BATT_VOLT_OFFSETS = 116,  // Read Battery Voltage Offsets
  SET_CUR_BLANKING_PCTS = 117,   // Set Current Blanking Percentages
  READ_CUR_BLANKING_PCTS = 118,  // Read Current Blanking Percentages
  DRIVE_M1_POS = 119,            // Drive M1 with Position
  DRIVE_M2_POS = 120,            // Drive M2 with Position
  DRIVE_M1_M2_POS = 121,         // Drive M1/M2 with Position
  DRIVE_M1_SPD_POS = 122,        // Drive M1 with Speed and Position
  DRIVE_M2_SPD_POS = 123,        // Drive M2 with Speed and Position
  DRIVE_M1_M2_SPD_POS = 124,     // Drive M1/M2 with Speed and Position

  SET_M1_MAX_CURRENT = 133,   // Set M1 Maximum Current
  SET_M2_MAX_CURRENT = 134,   // Set M2 Maximum Current
  READ_M1_MAX_CURRENT = 135,  // Read M1 Maximum Current
  READ_M2_MAX_CURRENT = 136,  // Read M2 Maximum Current

  SET_PWM_MODE = 148,   // Set PWM Mode
  READ_PWM_MODE = 149,  // Read PWM Mode

  READ_USER_EEPROM_LOC = 252,   // Read User EEPROM Memory Location
  WRITE_USER_EEPROM_LOC = 253,  // Write User EEPROM Memory Location

  NONE = 255  // No command
};

// Template struct used as the basis for all RoboClaw commands
template<Command ReadCommand, Command WriteCommand, typename ... Args>
struct Request
{
  Request() = default;
  explicit Request(Args &&... args) {fields = std::forward_as_tuple(args ...);}
  // Tuple of all types required for serialization
  using ArgsTuple = std::tuple<Args...>;

  // The command to read data from the roboclaw
  static constexpr uint8_t read_command = uint8_t(ReadCommand);

  // The command to write data to the roboclaw
  static constexpr uint8_t write_command = uint8_t(WriteCommand);

  static bool canRead() {return ReadCommand != Command::NONE;}

  static bool canWrite() {return WriteCommand != Command::NONE;}

  ArgsTuple fields;
};

using FirmwareVersion = Request<Command::READ_FIRMWARE_VER, Command::NONE, std::string>;

// Sets or gets the serial communication timeout in 100ms increments.
// When serial bytes are received in the time specified both motors
// will stop automatically. Range is 0 to 25.5 seconds (0 to 255 in
// 100ms increments)
using SerialTimeout = Request<Command::READ_SERIAL_TIMEOUT, Command::SET_SERIAL_TIMEOUT, uint8_t>;

// Read the main battery voltage level connected to B+ and B-terminals.
// The voltage is returned in 10ths of a volt(eg 300 = 30v)
using MainBatteryVoltage = Request<Command::READ_MAIN_BATT_VOLT, Command::NONE, uint16_t>;

using MainBatteryVoltageCutoff =
  Request<Command::READ_MAIN_BATT_VOLT_SET, Command::SET_MAIN_BATT_VOLT, uint8_t, uint8_t>;

// Read M1 and M2 encoder counters. Quadrature encoders have a range
// of 0 to 4,294,967,295. Absolute encoder values are converted from
// an analog voltage into a value from 0 to 2047 for the full 2V
// analog range.
using EncoderCounters = Request<Command::READ_M1_M2_ENC, Command::NONE, int32_t, int32_t>;

// Several motor and quadrature combinations can be used with RoboClaw.
// In some cases the default PID values will need to be tuned for the
// systems being driven. This gives greater flexibility in what motor and
// encoder combinations can be used. The RoboClaw PID system consists of
// four constants starting with QPPS, P = Proportional, I= Integral and
// D= Derivative. Each of these values is represented by 4 bytes.
using VelocityPIDConstantsM1 = Request<
  Command::READ_M1_VEL_PID_CONST, Command::SET_M1_VEL_PID_CONST, int32_t, int32_t, int32_t,
  int32_t>;

using VelocityPIDConstantsM2 = Request<
  Command::READ_M2_VEL_PID_CONST, Command::SET_M2_VEL_PID_CONST, int32_t, int32_t, int32_t,
  int32_t>;

// Drive M1 and M2 in the same command using a signed speed value. The
// sign indicates which direction the motor will turn. This command is
// used to drive both motors by quad pulses per second. Different
// quadrature encoders will have different rates at which they generate
// the incoming pulses. The values used will differ from one encoder to
// another. Once a value is sent the motor will begin to accelerate as
// fast as possible until the rate defined is reached.
using DriveM1M2WithSignedSpeed =
  Request<Command::NONE, Command::DRIVE_M1_M2_SGN_SPD, int32_t, int32_t>;

// Add additional commands here. See BasicMicro user manual for examples
// https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf

}  // namespace roboclaw_serial
