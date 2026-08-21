cmake_minimum_required(VERSION 3.16)
project(bot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(bot_hardware SHARED
  src/diffdrive_microros.cpp
)
target_compile_features(bot_hardware PUBLIC cxx_std_17)
target_include_directories(bot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/bot_hardware>
)
ament_target_dependencies(bot_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export the plugin to pluginlib
pluginlib_export_plugin_description_file(hardware_interface bot_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/bot_hardware
)
install(
  TARGETS bot_hardware
  EXPORT export_bot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_bot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
