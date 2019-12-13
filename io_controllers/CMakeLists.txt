cmake_minimum_required(VERSION 2.8.3)
project(io_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS hardware_state_command_interfaces realtime_tools roscpp hardware_interface pluginlib controller_interface std_msgs)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS hardware_state_command_interfaces realtime_tools roscpp hardware_interface pluginlib controller_interface std_msgs
  INCLUDE_DIRS include
  LIBRARIES io_controllers
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(io_controllers
    src/digital_output_command_controller.cpp
    src/digital_input_state_controller.cpp
    )
target_link_libraries(io_controllers ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(digital_output_command_controller_test
                    test/digital_output_command_controller.test
                    test/digital_output_command_controller_test.cpp)
  target_link_libraries(digital_output_command_controller_test io_controllers)
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES io_controllers_plugin.xml
              joint_state_controller.yaml
              joint_state_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
