cmake_minimum_required(VERSION 3.22.1)
project(ddt_msgs LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
        "msg/JointControlCommand.msg"
        "msg/DockStatus.msg"
        "msg/UserCommand.msg"
        "msg/CentroidState.msg"
        "msg/RigidBody.msg"
        "msg/CrsfTransferData.msg"
        "msg/DockTransferMessage.msg"
        "msg/PMSCommand.msg"
)

set(SRV_FILES
        "srv/SendDockTransfer.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        ${SRV_FILES}
        DEPENDENCIES std_msgs unique_identifier_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
