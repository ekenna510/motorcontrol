cmake_minimum_required(VERSION 3.5)
project(motorcontrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wextra -Wpedantic  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(joy REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(
  include
)
#target_link_libraries(${PROJECT_NAME}_lib)

add_executable(motorcontrol src/motorcontrol.cpp  src/botconfig.cpp src/SlaveParse.cpp src/sensordata.cpp src/serial.cpp )
ament_target_dependencies(motorcontrol rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_msgs nav_msgs tf2_ros) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


################################################################################
# Install
################################################################################
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS motorcontrol
  DESTINATION lib/${PROJECT_NAME}
)



################################################################################
# Macro for ament package
################################################################################


ament_export_include_directories(include)
ament_package()
