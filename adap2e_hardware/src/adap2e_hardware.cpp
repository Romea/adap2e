// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


// std
#include <fstream>
#include <memory>
#include <thread>
#include <string>

// romea
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

// ros
#include "rclcpp/rclcpp.hpp"

// local
#include "adap2e_hardware/adap2e_hardware.hpp"

namespace
{
const double WHEEL_LINEAR_SPEED_EPSILON = 0.01;
const double WHEEL_STEERING_ANGLE_EPSILON = 0.03;

const uint32_t FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID = 0x15;
const uint32_t REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID = 0x16;
const uint32_t FRONT_WHEEL_STEERING_ANGLES_COMMAND_ID = 0x17;
const uint32_t REAR_WHEEL_STEERING_ANGLES_COMMAND_ID = 0x18;

const uint32_t FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID = 0x25;
const uint32_t REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID = 0x23;
const uint32_t FRONT_WHEEL_STEERING_ANGLES_MEASUREMENT_ID = 0x26;
const uint32_t REAR_WHEEL_STEERING_ANGLES_MEASUREMENT_ID = 0x27;

const uint32_t START_STOP_ID = 0x10;

const std::chrono::milliseconds TIMEOUT(5);
}  //  namespace


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
Adap2eHardware::Adap2eHardware()
: HardwareSystemInterface4WS4WD("Adap2eHardware"),
  can_receiver_thread_(nullptr),
  can_receiver_thread_run_(false),
  can_sender_("can0"),
  can_receiver_("can0"),
  sended_frame_data_(),
  received_frame_data_(),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
  front_left_wheel_steering_angle_measure_(0),
  front_right_wheel_steering_angle_measure_(0),
  rear_left_wheel_steering_angle_measure_(0),
  rear_right_wheel_steering_angle_measure_(0),
  front_left_wheel_linear_speed_measure_(0),
  front_right_wheel_linear_speed_measure_(0),
  rear_left_wheel_linear_speed_measure_(0),
  rear_right_wheel_linear_speed_measure_(0),
  front_left_wheel_steering_angle_command_(0),
  front_right_wheel_steering_angle_command_(0),
  rear_left_wheel_steering_angle_command_(0),
  rear_right_wheel_steering_angle_command_(0),
  front_left_wheel_linear_speed_command_(0),
  front_right_wheel_linear_speed_command_(0),
  rear_left_wheel_linear_speed_command_(0),
  rear_right_wheel_linear_speed_command_(0)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
Adap2eHardware::~Adap2eHardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::connect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("Adap2eHardware"), "Init communication with robot");

  send_null_command_();
  start_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::disconnect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("Adap2eHardware"), "Close communication with robot");

  send_null_command_();
  stop_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Adap2eHardware"), "load_info");

  try {
    // front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    // rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    front_wheel_radius_ = get_front_wheel_radius(hardware_info);
    rear_wheel_radius_ = get_rear_wheel_radius(hardware_info);
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("Adap2eHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type Adap2eHardware::read()
#else
hardware_interface::return_type Adap2eHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  //  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Read data from robot");

  set_hardware_state_();

  // std::cout << "wheels speeds " <<
  //   front_left_wheel_linear_speed_measure_.load() << " " <<
  //   front_right_wheel_linear_speed_measure_.load() << " " <<
  //   rear_left_wheel_linear_speed_measure_.load() << " " <<
  //   rear_right_wheel_linear_speed_measure_.load() << std::endl;

  // std::cout << "wheels angless " <<
  //   front_left_wheel_steering_angle_measure_.load() << " " <<
  //   front_right_wheel_steering_angle_measure_.load() << " " <<
  //   rear_left_wheel_steering_angle_measure_.load() << " " <<
  //   rear_right_wheel_steering_angle_measure_.load() << std::endl;

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type Adap2eHardware::write()
# else
hardware_interface::return_type Adap2eHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  //  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Send command to robot");

  get_hardware_command_();


  if (is_drive_enable_()) {
    // std::cout << " send command " << std::endl;
    send_command_();

    // std::cout << "wheels speeds command " << front_left_wheel_linear_speed_command_ << " " <<
    //   front_right_wheel_linear_speed_command_ << " " << rear_left_wheel_linear_speed_command_ <<
    //   " " << rear_right_wheel_linear_speed_command_ << std::endl;

    // std::cout << "wheels angless command " <<
    //   front_left_wheel_steering_angle_command_ << " " <<
    //   front_right_wheel_steering_angle_command_ << " " <<
    //   rear_left_wheel_steering_angle_command_ << " " <<
    //   rear_right_wheel_steering_angle_command_ << std::endl;
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void Adap2eHardware::set_hardware_state_()
{
  core::HardwareState4WS4WD state;

  state.frontLeftWheelSteeringAngle = front_left_wheel_steering_angle_measure_;
  state.frontRightWheelSteeringAngle = front_right_wheel_steering_angle_measure_;
  state.rearLeftWheelSteeringAngle = rear_left_wheel_steering_angle_measure_;
  state.rearRightWheelSteeringAngle = rear_right_wheel_steering_angle_measure_;

  // state.frontLeftWheelSpinningMotion.velocity =
  //   front_left_wheel_linear_speed_measure_ / front_wheel_radius_;
  // state.frontRightWheelSpinningMotion.velocity =
  //   front_right_wheel_linear_speed_measure_ / front_wheel_radius_;
  // state.rearLeftWheelSpinningMotion.velocity =
  //   rear_left_wheel_linear_speed_measure_ / rear_wheel_radius_;
  // state.rearRightWheelSpinningMotion.velocity =
  //   rear_right_wheel_linear_speed_measure_ / rear_wheel_radius_;

  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_linear_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_linear_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = rear_left_wheel_linear_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = rear_right_wheel_linear_speed_measure_;

  hardware_interface_->set_feedback(state);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::get_hardware_command_()
{
  core::HardwareCommand4WS4WD command = hardware_interface_->get_hardware_command();

  front_left_wheel_steering_angle_command_ = command.frontLeftWheelSteeringAngle;
  front_right_wheel_steering_angle_command_ = command.frontRightWheelSteeringAngle;
  rear_left_wheel_steering_angle_command_ = command.rearLeftWheelSteeringAngle;
  rear_right_wheel_steering_angle_command_ = command.rearRightWheelSteeringAngle;

  // front_left_wheel_linear_speed_command_ =
  //   command.frontLeftWheelSpinningSetPoint * front_wheel_radius_;
  // front_right_wheel_linear_speed_command_ =
  //   command.frontRightWheelSpinningSetPoint * front_wheel_radius_;
  // rear_left_wheel_linear_speed_command_ =
  //   command.rearLeftWheelSpinningSetPoint * rear_wheel_radius_;
  // rear_right_wheel_linear_speed_command_ =
  //   command.rearRightWheelSpinningSetPoint * rear_wheel_radius_;

  front_left_wheel_linear_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_linear_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_linear_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_linear_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_command_()
{
  return send_start_() &&  // why send start at each command?
         send_front_wheel_speeds_() &&
         send_rear_wheel_speeds_() &&
         send_front_wheel_angles_() &&
         send_rear_wheel_angles_();
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_null_command_()
{
  memset(sended_frame_data_.data(), 0, 8);
  return send_data_(FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID) &&
         send_data_(REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID) &&
         send_data_(FRONT_WHEEL_STEERING_ANGLES_COMMAND_ID) &&
         send_data_(REAR_WHEEL_STEERING_ANGLES_COMMAND_ID);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::receive_data_()
{
  while (rclcpp::ok() && can_receiver_thread_run_) {
    try {
      drivers::socketcan::CanId receive_id = can_receiver_.
        receive(received_frame_data_.data(), TIMEOUT);

      switch (receive_id.identifier()) {
        case FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID:
          decode_front_wheel_speeds_();
          break;
        case REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID:
          decode_rear_wheel_speeds_();
          break;
        case FRONT_WHEEL_STEERING_ANGLES_MEASUREMENT_ID:
          decode_front_wheel_angles_();
          break;
        case REAR_WHEEL_STEERING_ANGLES_MEASUREMENT_ID:
          decode_rear_wheel_angles_();
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("Adap2eHardware"), "Error receiving CAN message");
    }
  }
}


//-----------------------------------------------------------------------------
bool Adap2eHardware::send_data_(uint32_t id)
{
  try {
    drivers::socketcan::CanId can_id(id, 0, 8);
    can_sender_.send(sended_frame_data_.data(), 8, can_id, TIMEOUT);
    return true;
  } catch (drivers::socketcan::SocketCanTimeout & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("Adap2eHardware"),
      "Send can data" << std::hex << id << " : timeout");
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("Adap2eHardware"),
      "Send can data" << std::hex << id << " : " << e.what());
  }
  return false;
}

//-----------------------------------------------------------------------------
void Adap2eHardware::encode_odo_data_(
  float left_data,
  float right_data)
{
  std::memcpy(&sended_frame_data_[0], &left_data, sizeof(left_data));
  std::memcpy(&sended_frame_data_[4], &right_data, sizeof(right_data));
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_front_wheel_speeds_()
{
  encode_odo_data_(
    front_left_wheel_linear_speed_command_,
    front_right_wheel_linear_speed_command_);
  return send_data_(FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_rear_wheel_speeds_()
{
  encode_odo_data_(
    rear_left_wheel_linear_speed_command_,
    rear_right_wheel_linear_speed_command_);
  return send_data_(REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_front_wheel_angles_()
{
  encode_odo_data_(
    -front_left_wheel_steering_angle_command_,
    -front_right_wheel_steering_angle_command_);
  return send_data_(FRONT_WHEEL_STEERING_ANGLES_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_rear_wheel_angles_()
{
  encode_odo_data_(
    -rear_left_wheel_steering_angle_command_,
    -rear_right_wheel_steering_angle_command_);
  return send_data_(REAR_WHEEL_STEERING_ANGLES_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::send_start_()
{
  sended_frame_data_[0] = 2;  // Demande départ mode autonome
  sended_frame_data_[1] = 1;  // 1=Boucle ouverte
  return send_data_(START_STOP_ID);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::decode_odo_data_(
  std::atomic<float> & left_data,
  std::atomic<float> & right_data)
{
  left_data.store(reinterpret_cast<float *>(received_frame_data_.data())[0]);
  right_data.store(reinterpret_cast<float *>(received_frame_data_.data())[1]);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::decode_front_wheel_speeds_()
{
  decode_odo_data_(
    front_left_wheel_linear_speed_measure_,
    front_right_wheel_linear_speed_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("Adap2eHardware"),
    "ID = " << FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID <<
      " front speeds : left " << front_left_wheel_linear_speed_measure_ <<
      " , right " << front_right_wheel_linear_speed_measure_);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::decode_rear_wheel_speeds_()
{
  decode_odo_data_(
    rear_left_wheel_linear_speed_measure_,
    rear_right_wheel_linear_speed_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("Adap2eHardware"),
    "ID = " << REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID <<
      " rear speeds : left " << rear_left_wheel_linear_speed_measure_ <<
      " , right " << rear_right_wheel_linear_speed_measure_);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::decode_front_wheel_angles_()
{
  decode_odo_data_(
    front_left_wheel_steering_angle_measure_,
    front_right_wheel_steering_angle_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("Adap2eHardware"),
    "ID = " << FRONT_WHEEL_STEERING_ANGLES_MEASUREMENT_ID <<
      " front angles : left " << front_left_wheel_steering_angle_measure_ <<
      " , right " << front_right_wheel_steering_angle_measure_);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::decode_rear_wheel_angles_()
{
  decode_odo_data_(
    rear_left_wheel_steering_angle_measure_,
    rear_right_wheel_steering_angle_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("Adap2eHardware"),
    "ID = " << REAR_WHEEL_STEERING_ANGLES_MEASUREMENT_ID <<
      " rear angles : left " << rear_left_wheel_steering_angle_measure_ <<
      " , right " << rear_right_wheel_steering_angle_measure_);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::start_can_receiver_thread_()
{
  can_receiver_thread_run_ = true;
  can_receiver_thread_ = std::make_unique<std::thread>(&Adap2eHardware::receive_data_, this);
}

//-----------------------------------------------------------------------------
void Adap2eHardware::stop_can_receiver_thread_()
{
  can_receiver_thread_run_ = false;
  if (can_receiver_thread_->joinable()) {
    can_receiver_thread_->join();
  }
}

//-----------------------------------------------------------------------------
bool Adap2eHardware::is_drive_enable_()const
{
  float speed_measure = 0.25 * (front_left_wheel_linear_speed_measure_ +
    front_right_wheel_linear_speed_measure_ +
    rear_left_wheel_linear_speed_measure_ +
    rear_right_wheel_linear_speed_measure_);

  float speed_command = 0.25 * (front_left_wheel_linear_speed_command_ +
    front_right_wheel_linear_speed_command_ +
    rear_left_wheel_linear_speed_command_ +
    rear_right_wheel_linear_speed_command_);

  return !(std::abs(front_left_wheel_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(front_left_wheel_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(front_right_wheel_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(front_right_wheel_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_left_wheel_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_left_wheel_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_right_wheel_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_right_wheel_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(speed_measure) < WHEEL_LINEAR_SPEED_EPSILON &&
         std::abs(speed_command) < WHEEL_LINEAR_SPEED_EPSILON);
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void Adap2eHardware::open_log_file_()
{
  debug_file_.open(
    std::string("adap2e.dat").c_str(),
    std::fstream::in | std::fstream::out | std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void Adap2eHardware::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << "# time, ";
    debug_file_ << " FLS, " << " FRS, ";
    debug_file_ << " RLS, " << " RRS, ";
    debug_file_ << " FLA, " << " FRA, ";
    debug_file_ << " RLA, " << " RRA, ";
    debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
    debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
    debug_file_ << " FLA_cmd, " << " FRA_cmd, ";
    debug_file_ << " RLA_cmd, " << " RRA_cmd, " << "\n";
  }
}

//-----------------------------------------------------------------------------
void Adap2eHardware::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count() << " ";
    debug_file_ << front_left_wheel_linear_speed_measure_ << " " <<
      front_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_measure_ << " " <<
      rear_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << front_left_wheel_steering_angle_measure_ << " " <<
      front_right_wheel_steering_angle_measure_ << " ";
    debug_file_ << rear_left_wheel_steering_angle_measure_ << " " <<
      rear_right_wheel_steering_angle_measure_ << " ";
    debug_file_ << front_left_wheel_linear_speed_command_ << " " <<
      front_right_wheel_linear_speed_command_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_command_ << " " <<
      rear_right_wheel_linear_speed_command_ << " ";
    debug_file_ << front_left_wheel_steering_angle_command_ << " " <<
      front_right_wheel_steering_angle_command_ << " ";
    debug_file_ << rear_left_wheel_steering_angle_command_ << " " <<
      rear_right_wheel_steering_angle_command_ << " \n";
  }
}
#endif

}  // namespace ros2
}  // namespace romea


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::Adap2eHardware, hardware_interface::SystemInterface)
