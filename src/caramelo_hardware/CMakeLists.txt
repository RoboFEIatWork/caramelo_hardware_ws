cmake_minimum_required(VERSION 3.8)
project(caramelo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
include(CheckIncludeFileCXX)
check_include_file_cxx(pigpiod_if2.h HAVE_PIGPIOD_IF2_H)
check_include_file_cxx(pigpio.h HAVE_PIGPIO_H)
find_library(PIGPIOD_IF2_LIB pigpiod_if2)
find_library(PIGPIO_LIB pigpio)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mobile_base_hw_interface.cpp
  src/mobile_base_hw_parser.cpp
  src/maxon_gpio_driver.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

if(HAVE_PIGPIOD_IF2_H AND HAVE_PIGPIO_H AND PIGPIOD_IF2_LIB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_PIGPIO=1)
  target_link_libraries(${PROJECT_NAME} ${PIGPIOD_IF2_LIB})
  if(PIGPIO_LIB)
    target_link_libraries(${PROJECT_NAME} ${PIGPIO_LIB})
  endif()
  message(STATUS "caramelo_hardware: pigpio backend enabled.")
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_PIGPIO=0)
  message(WARNING "caramelo_hardware: pigpio headers/libs not found. GPIO backend disabled for this build.")
endif()

pluginlib_export_plugin_description_file(hardware_interface caramelo_hardware_interface.xml)

install (
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES caramelo_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
