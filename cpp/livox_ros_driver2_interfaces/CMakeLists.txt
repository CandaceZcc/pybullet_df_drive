# 阶段四 D：从锁定的 Livox 官方消息定义生成 ROS 2 CustomMsg 类型。
cmake_minimum_required(VERSION 3.28)
project(livox_ros_driver2 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT DEFINED STAGE4_LIVOX_MSG_SOURCE OR
   NOT EXISTS "${STAGE4_LIVOX_MSG_SOURCE}/msg/CustomMsg.msg" OR
   NOT EXISTS "${STAGE4_LIVOX_MSG_SOURCE}/msg/CustomPoint.msg")
  message(FATAL_ERROR "Phase-0 ROS Bridge requires the locked Livox message source")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "${STAGE4_LIVOX_MSG_SOURCE}:msg/CustomPoint.msg"
  "${STAGE4_LIVOX_MSG_SOURCE}:msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(STAGE4_LIVOX_CPP_TYPESUPPORT ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_library(stage4_livox_ros2_interfaces INTERFACE)
target_link_libraries(stage4_livox_ros2_interfaces INTERFACE "${STAGE4_LIVOX_CPP_TYPESUPPORT}")
ament_package()
