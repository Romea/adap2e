cmake_minimum_required(VERSION 3.5)
project(adap2e_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/adap2e_bringup.py)

install(PROGRAMS
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
