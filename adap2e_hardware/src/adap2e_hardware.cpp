#include "adap2e_hardware/adap2e_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
Adap2eHardware::Adap2eHardware():
  HardwareSystemInterface4WS4WD()
{

}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Read data from robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Send command to robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::connect()
{
  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Init communication with robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type Adap2eHardware::disconnect()
{
  RCLCPP_INFO(rclcpp::get_logger("Adap2eHardware"), "Close communication with robot");
  return hardware_interface::return_type::OK;
}

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::Adap2eHardware, hardware_interface::SystemInterface)
