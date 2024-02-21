cmake_minimum_required(VERSION 3.8)
project(icm20948_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(Fusion)

### Interfaces
### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RPY.msg"
        "msg/RawData.msg"
        "msg/DebugFusion.msg"
        DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

### Executable
add_executable(icm20948
        src/icm20948_node.cpp
        src/icm20948.cpp
)

ament_target_dependencies(icm20948
        rclcpp
        rosidl_default_generators
        std_msgs
        geometry_msgs
)

target_link_libraries(icm20948
        "${cpp_typesupport_target}"
        Fusion
)

### Includes
target_include_directories(icm20948 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS icm20948
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS Fusion
        DESTINATION lib/
)

ament_package()
