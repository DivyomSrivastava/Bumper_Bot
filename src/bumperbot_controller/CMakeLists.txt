cmake_minimum_required(VERSION 3.5)
project(bumperbot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

set(CPP_TARGETS "")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_controller.cpp)
  add_executable(simple_controller src/simple_controller.cpp)
  ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2)
  target_link_libraries(simple_controller Eigen3::Eigen)
  list(APPEND CPP_TARGETS simple_controller)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/noisy_controller.cpp)
  add_executable(noisy_controller src/noisy_controller.cpp)
  ament_target_dependencies(noisy_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)
  list(APPEND CPP_TARGETS noisy_controller)
endif()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY include DESTINATION include)

if(CPP_TARGETS)
  install(TARGETS ${CPP_TARGETS} DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  ${PROJECT_NAME}/noisy_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()