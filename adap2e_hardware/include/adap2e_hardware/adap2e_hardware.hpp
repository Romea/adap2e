#ifndef ADAP2E_HARDWARE_HPP_
#define ADAP2E_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <rclcpp/macros.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>

namespace romea
{

class Adap2eHardware : public HardwareSystemInterface4WS4WD
{  

public:

  RCLCPP_SHARED_PTR_DEFINITIONS(Adap2eHardware);

  Adap2eHardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;

private:

  virtual hardware_interface::return_type connect_() override;

  virtual hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
      const hardware_interface::HardwareInfo & hardware_info) override ;


  void encode_odo_data_(float left_data,
                        float right_data);

  void decode_odo_data_(std::atomic<float> & left_data,
                        std::atomic<float> & right_data);


  bool send_data_(uint32_t id);

  bool send_front_wheel_speeds_();

  bool send_rear_wheel_speeds_();

  bool send_front_wheel_angles_();

  bool send_rear_wheel_angles_();

  bool send_command_();

  bool send_null_command_();

  bool send_start_();


  void receive_data_();

  void decode_front_wheel_speeds_();

  void decode_rear_wheel_speeds_();

  void decode_front_wheel_angles_();

  void decode_rear_wheel_angles_();

  bool is_drive_enable_() const;


  void start_can_receiver_thread_();

  void stop_can_receiver_thread_();

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:

  std::unique_ptr<std::thread> can_receiver_thread_;
  std::atomic<bool> can_receiver_thread_run_;;

  drivers::socketcan::SocketCanSender can_sender_;
  drivers::socketcan::SocketCanReceiver can_receiver_;
  std::array<uint8_t,8> sended_frame_data_;
  std::array<uint8_t,8> received_frame_data_;

  float front_wheel_radius_;
  float rear_wheel_radius_;

  std::atomic<float> front_left_wheel_angle_measure_;
  std::atomic<float> front_right_wheel_angle_measure_;
  std::atomic<float> rear_left_wheel_angle_measure_;
  std::atomic<float> rear_right_wheel_angle_measure_;
  std::atomic<float> front_left_wheel_speed_measure_;
  std::atomic<float> front_right_wheel_speed_measure_;
  std::atomic<float> rear_left_wheel_speed_measure_;
  std::atomic<float> rear_right_wheel_speed_measure_;

  float front_left_wheel_angle_command_;
  float front_right_wheel_angle_command_;
  float rear_left_wheel_angle_command_;
  float rear_right_wheel_angle_command_;
  float front_left_wheel_speed_command_;
  float front_right_wheel_speed_command_;
  float rear_left_wheel_speed_command_;
  float rear_right_wheel_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif

};

}

#endif
