cmake_minimum_required(VERSION 3.8)
project(voiture2a_motors_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MotorsState.msg"
        "msg/Engine.msg"
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(motors_node src/motors_node.cpp src/motors.cpp)
ament_target_dependencies(motors_node
        rclcpp
        geometry_msgs
        rosidl_default_generators
)
target_link_libraries(motors_node "${cpp_typesupport_target}" i2c)

### Includes
target_include_directories(motors_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS motors_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
