#ifndef ADAP2E_HARDWARE_HPP_
#define ADAP2E_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <rclcpp/macros.hpp>

namespace romea
{

class Adap2eHardware : public HardwareSystemInterface4WS4WD
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(Adap2eHardware);

  Adap2eHardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;

  virtual hardware_interface::return_type connect() override;

  virtual hardware_interface::return_type disconnect() override;

};

}

#endif
