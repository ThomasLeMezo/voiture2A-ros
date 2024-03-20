cmake_minimum_required(VERSION 3.8)
project(icm20948_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_CXX_STANDARD 20)

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
        i2c
)
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

message("CMAKE_HOST_SYSTEM_PROCESSOR = " ${CMAKE_HOST_SYSTEM_PROCESSOR})
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        find_package(rosbag2_cpp REQUIRED)

        ## Test Fusion
        add_executable(test_fusion
                src/test_fusion.cpp
        )
        ament_target_dependencies(test_fusion
                rclcpp
                rosidl_default_generators
                std_msgs
                geometry_msgs
        )
        target_link_libraries(test_fusion
                "${cpp_typesupport_target}"
                Fusion
        )
        target_include_directories(test_fusion PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        )

        ## Fusion Analysis
        add_executable(fusion_analysis
                src/fusion_analysis.cpp
        )
        ament_target_dependencies(fusion_analysis
                rclcpp
                rosidl_default_generators
                std_msgs
                geometry_msgs
                rosbag2_cpp
        )
        target_link_libraries(fusion_analysis
                "${cpp_typesupport_target}"
                Fusion
        )
        target_include_directories(fusion_analysis PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        )

        ## Magnetic calibration

        find_package(VTK 9.0 COMPONENTS
                CommonCore
                CommonDataModel
                FiltersSources
                InteractionStyle
                RenderingCore
                RenderingOpenGL2

                ChartsCore
                CommonColor
                RenderingContext2D
                RenderingContextOpenGL2
                RenderingFreeType
                RenderingGL2PSOpenGL2
                ViewsContext2D

                CommonTransforms
                RenderingAnnotation
                CommonComputationalGeometry
                REQUIRED)

        add_executable(magnetic_calibration
                src/magnetic_calibration/magnetic_calibration.cpp
                src/magnetic_calibration/magnetic_paving.cpp
        )
        ament_target_dependencies(magnetic_calibration
                rclcpp
                rosidl_default_generators
                rosbag2_cpp
        )
        target_link_libraries(magnetic_calibration
                "${cpp_typesupport_target}"
                ${VTK_LIBRARIES}
        )
        target_include_directories(magnetic_calibration PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
                ${VTK_INCLUDE_DIRS}
        )

        vtk_module_autoinit(
                TARGETS magnetic_calibration
                MODULES ${VTK_LIBRARIES}
        )

        ## --- Install ---
        install(TARGETS fusion_analysis test_fusion magnetic_calibration
                DESTINATION lib/${PROJECT_NAME}
        )

endif()

ament_package()
