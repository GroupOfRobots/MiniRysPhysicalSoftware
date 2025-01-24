cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(minirys_ros2
  LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rttest REQUIRED)
find_package(tlsf_cpp REQUIRED)

find_package(sbc-linux-interfaces REQUIRED)
find_package(minirys_msgs REQUIRED)
find_package(minirys_sensors REQUIRED)
find_package(vl53l1x-linux REQUIRED)
find_package(l6470-linux REQUIRED)
find_package(ldlidar_stl_ros2 REQUIRED)
find_package(pid_regulator REQUIRED)


include_directories(include)

######
# Node libraries
######
## VRs
# Battery
add_library(BatteryNode
  src/nodes/BatteryNode.cpp
)
ament_target_dependencies(BatteryNode
  rclcpp
  sbc-linux-interfaces
  minirys_msgs
  minirys_sensors
)
target_link_libraries(BatteryNode
  sbc-linux-interfaces
  minirys_sensors
)
# Distance
add_library(DistanceNode
  src/nodes/DistanceNode.cpp
)
ament_target_dependencies(DistanceNode
  rclcpp
  sbc-linux-interfaces
  vl53l1x-linux
  sensor_msgs
)
target_link_libraries(DistanceNode
  sbc-linux-interfaces
  vl53l1x-linux
)
# IMU
add_library(IMUNode
  src/nodes/IMUNode.cpp
)
ament_target_dependencies(IMUNode
  rclcpp
  sbc-linux-interfaces
  minirys_sensors
  sensor_msgs
  minirys_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(IMUNode
  sbc-linux-interfaces
  minirys_sensors

)
# Temperature
add_library(TemperatureNode
  src/nodes/TemperatureNode.cpp
)
ament_target_dependencies(TemperatureNode
  rclcpp
  sbc-linux-interfaces
  minirys_sensors
  std_msgs
)
target_link_libraries(TemperatureNode
  sbc-linux-interfaces
  minirys_sensors
)
## VEs
# Fan
add_library(FanNode
  src/nodes/FanNode.cpp
)
ament_target_dependencies(FanNode
  rclcpp
  sbc-linux-interfaces
  std_msgs
)
target_link_libraries(FanNode
  sbc-linux-interfaces
)
# Servo
add_library(ServoNode
  src/nodes/ServoNode.cpp
)
ament_target_dependencies(ServoNode
  rclcpp
  sbc-linux-interfaces
  std_msgs
)
target_link_libraries(ServoNode
  sbc-linux-interfaces
)
# Motors
add_library(MotorsNode
  src/nodes/MotorsNode.cpp
)
ament_target_dependencies(MotorsNode
  rclcpp
  sbc-linux-interfaces
  l6470-linux
  std_msgs
  minirys_msgs
  sensor_msgs
)
target_link_libraries(MotorsNode
  sbc-linux-interfaces
  l6470-linux
)
## CS
# Communication
add_library(CommunicationNode
  src/nodes/CommunicationNode.cpp
)

target_link_libraries(CommunicationNode
  Boost::system 
  Boost::filesystem
)

ament_target_dependencies(CommunicationNode
  rclcpp
  std_msgs
  sensor_msgs
  minirys_msgs
)
# Fan regulator
add_library(FanRegulatorNode
  src/nodes/FanRegulatorNode.cpp
)
ament_target_dependencies(FanRegulatorNode
  rclcpp
  std_msgs
  minirys_msgs
)
# Servo controller
add_library(ServoControllerNode
  src/nodes/ServoControllerNode.cpp
)
ament_target_dependencies(ServoControllerNode
  rclcpp
  std_msgs
  minirys_msgs
)
# Joystick controller
add_library(JoyconReceiverNode
        src/nodes/JoyconReceiverNode.cpp
        )
ament_target_dependencies(JoyconReceiverNode
        rclcpp
        geometry_msgs
        )
# Motors controller
add_library(MotorsControllerNode
  src/nodes/MotorsControllerNode.cpp
  src/helpers/TimeMeasure.cpp
)
ament_target_dependencies(MotorsControllerNode
  rclcpp
  std_msgs
  geometry_msgs
  minirys_msgs
  pid_regulator
)
# Odometry
add_library(OdometryNode
  src/nodes/OdometryNode.cpp
)
ament_target_dependencies(OdometryNode
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  minirys_msgs
)

## Helpers
add_library(RTTExecutor
  src/helpers/RTTExecutor.cpp
)
ament_target_dependencies(RTTExecutor
  rclcpp
  rttest
  minirys_msgs
  tlsf_cpp
)

add_library(LinearTestNode
  src/test/LinearTestNode.cpp
)
ament_target_dependencies(LinearTestNode
  rclcpp
  nav_msgs
  geometry_msgs
)

######
# Executables
######
add_executable(battery
  src/executables/battery.cpp
)
target_link_libraries(battery
  BatteryNode
  RTTExecutor
)

add_executable(odometry
  src/executables/odometry.cpp
)
target_link_libraries(odometry
  OdometryNode
  RTTExecutor
)

add_executable(imu
  src/executables/imu.cpp
)
target_link_libraries(imu
  IMUNode
  RTTExecutor
)

add_executable(temperature
  src/executables/temperature.cpp
)
target_link_libraries(temperature
  TemperatureNode
  RTTExecutor
)

add_executable(sensors
  src/executables/sensors.cpp
)
target_link_libraries(sensors
  BatteryNode
  IMUNode
  TemperatureNode
  RTTExecutor
)

add_executable(distance
  src/executables/distance.cpp
)
target_link_libraries(distance
  DistanceNode
  RTTExecutor
)

add_executable(motors
  src/executables/motors.cpp
)
target_link_libraries(motors
  MotorsNode
  RTTExecutor
)

add_executable(fan
  src/executables/fan.cpp
)

add_executable(servo
  src/executables/servo.cpp
)
target_link_libraries(servo
  ServoNode
  RTTExecutor
)
target_link_libraries(fan
  FanNode
  RTTExecutor
)

add_executable(joycon_receiver_node
        src/executables/joycon_receiver_node.cpp
        )
target_link_libraries(joycon_receiver_node
        JoyconReceiverNode
        RTTExecutor
        )

# Main virtual effectors executable
add_executable(minirys_ve
  src/executables/minirys_ve.cpp
)
target_link_libraries(minirys_ve
  FanNode
  ServoNode
  MotorsNode
  RTTExecutor
)

# Main virtual receptors executable
add_executable(minirys_vr
  src/executables/minirys_vr.cpp
)
target_link_libraries(minirys_vr
  BatteryNode
  DistanceNode
  IMUNode
  TemperatureNode
  RTTExecutor
)

# Main control subsystem executable
add_executable(minirys_cs
  src/executables/minirys_cs.cpp
)
target_link_libraries(minirys_cs
  Boost::system 
  Boost::filesystem
  CommunicationNode 
  FanRegulatorNode
  MotorsControllerNode
  OdometryNode
  RTTExecutor
)
# Additional control subsystems executable - for LiDAR version of MiniRyś robot
add_executable(minirys_cs_lidar
  src/executables/minirys_cs_lidar.cpp
)
target_link_libraries(minirys_cs_lidar
  CommunicationNode
  FanRegulatorNode
  ServoControllerNode
  MotorsControllerNode
  OdometryNode
  RTTExecutor
)
# Additional control subsystems executables - for RT testing
add_executable(minirys_cs_rttest
  src/executables/minirys_cs_rttest.cpp
)
target_link_libraries(minirys_cs_rttest
  CommunicationNode
  FanRegulatorNode
  ServoControllerNode
  MotorsControllerNode
  OdometryNode
  RTTExecutor
)
add_executable(minirys_cs_rttest_rt
  src/executables/minirys_cs_rttest_rt.cpp
)
target_link_libraries(minirys_cs_rttest_rt
  CommunicationNode
  FanRegulatorNode
  ServoControllerNode
  MotorsControllerNode
  OdometryNode
  RTTExecutor
)

add_executable(linear_test
  src/executables/linear_test.cpp
)
target_link_libraries(linear_test
  LinearTestNode
  RTTExecutor
)

# Install executables
install(
  TARGETS
    battery
    distance
    fan
    servo
    joycon_receiver_node
    imu
    motors
    odometry
    sensors
    temperature
    minirys_ve
    minirys_vr
    minirys_cs
    minirys_cs_lidar
    minirys_cs_rttest
    minirys_cs_rttest_rt
    linear_test
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY lib/
#   DESTINATION lib
# )

# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# set(CMAKE_INSTALL_RPATH "$/home/minirys/minirys_ws/install/sbc-linux-interfaces/lib")
# set(CMAKE_INSTALL_RPATH "$/home/minirys/minirys_ws/install/minirys_ros2/lib")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
