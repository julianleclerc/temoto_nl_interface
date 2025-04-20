cmake_minimum_required(VERSION 3.8)
project(temoto_nl_interface)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(temoto_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Add the nl_interface_msgs package
add_subdirectory(nl_interface_msgs)

# Verify Python package structure first
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/temoto_nl_interface/__init__.py)
  message(FATAL_ERROR "Python package is missing __init__.py file. Please create one.")
endif()

# Install Python package
ament_python_install_package(temoto_nl_interface)

# Define the scripts to install
set(PYTHON_SCRIPTS
  chat_interface
  umrf_planner
  error_handler
  memory
  display_interface
  cam_publisher
  fake_nav
)

# Install Python scripts as executables
foreach(script ${PYTHON_SCRIPTS})
  install(PROGRAMS
    "${CMAKE_CURRENT_SOURCE_DIR}/temoto_nl_interface/nodes/${script}.py"
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclpy temoto_msgs cv_bridge nl_interface_msgs)

ament_package()