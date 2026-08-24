# 阶段四 Phase-0：先注册 CTest 合同，再在 GREEN 阶段接入 raw eCAL C++ target。
cmake_minimum_required(VERSION 3.28)
project(slope_sim_stage4_phase0 VERSION 0.1.0 LANGUAGES NONE)

include(CTest)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT Python3_VERSION_MAJOR EQUAL 3 OR NOT Python3_VERSION_MINOR EQUAL 10)
  message(FATAL_ERROR "Phase-0 CTest requires the slope-sim Python 3.10 interpreter")
endif()
if(NOT BUILD_TESTING)
  message(FATAL_ERROR "Phase-0 contract target requires BUILD_TESTING=ON")
endif()
option(STAGE4_PHASE0_RED_ONLY "Configure only the pre-implementation RED target" OFF)
option(STAGE4_WITH_ROS "Build optional Jazzy ROS 2 downstream participants" OFF)
get_filename_component(STAGE4_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

add_test(
  NAME phase0_tools_report_frozen_abi
  COMMAND "${Python3_EXECUTABLE}" -m pytest -q
          "${STAGE4_ROOT}/tests/stage4/test_cpp_phase0_build.py"
)
set_tests_properties(
  phase0_tools_report_frozen_abi
  PROPERTIES
    ENVIRONMENT "STAGE4_PHASE0_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}"
    WORKING_DIRECTORY "${STAGE4_ROOT}"
)

if(STAGE4_PHASE0_RED_ONLY)
  return()
endif()

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 13 OR CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 14)
  message(FATAL_ERROR "Phase-0 requires GCC 13")
endif()
if(NOT DEFINED STAGE4_PROTOC_EXECUTABLE)
  message(FATAL_ERROR "Phase-0 requires STAGE4_PROTOC_EXECUTABLE")
endif()
set(STAGE4_MCAP_INCLUDE_DIR "" CACHE PATH "Locked MCAP C++ include directory")
if(NOT EXISTS "${STAGE4_MCAP_INCLUDE_DIR}/mcap/mcap.hpp")
  message(FATAL_ERROR "Phase-0 requires STAGE4_MCAP_INCLUDE_DIR from the locked MCAP source tree")
endif()

find_package(eCAL 6.1.1 EXACT REQUIRED CONFIG)
find_package(Protobuf 33.6.0 EXACT REQUIRED CONFIG)
find_package(OpenSSL 3 REQUIRED COMPONENTS Crypto)

if(NOT eCAL_VERSION VERSION_EQUAL 6.1.1)
  message(FATAL_ERROR "Phase-0 requires eCAL 6.1.1")
endif()
if(NOT Protobuf_VERSION VERSION_EQUAL 33.6.0)
  message(FATAL_ERROR "Phase-0 requires Protobuf 33.6.0")
endif()
# eCAL 会把 RelWithDebInfo 映射到 Release；按其明确映射读取同一套 protoc。
string(TOUPPER "${CMAKE_BUILD_TYPE}" STAGE4_IMPORTED_CONFIG)
set(STAGE4_IMPORTED_CONFIGS "${STAGE4_IMPORTED_CONFIG}")
set(STAGE4_IMPORTED_CONFIG_MAP "CMAKE_MAP_IMPORTED_CONFIG_${STAGE4_IMPORTED_CONFIG}")
if(DEFINED ${STAGE4_IMPORTED_CONFIG_MAP})
  list(PREPEND STAGE4_IMPORTED_CONFIGS ${${STAGE4_IMPORTED_CONFIG_MAP}})
endif()
foreach(STAGE4_IMPORTED_CONFIG IN LISTS STAGE4_IMPORTED_CONFIGS)
  string(TOUPPER "${STAGE4_IMPORTED_CONFIG}" STAGE4_IMPORTED_CONFIG)
  get_target_property(
    STAGE4_IMPORTED_PROTOC
    protobuf::protoc
    "IMPORTED_LOCATION_${STAGE4_IMPORTED_CONFIG}"
  )
  if(STAGE4_IMPORTED_PROTOC)
    break()
  endif()
endforeach()
if(NOT STAGE4_IMPORTED_PROTOC)
  message(FATAL_ERROR "Protobuf config must export protobuf::protoc")
endif()
file(REAL_PATH "${STAGE4_IMPORTED_PROTOC}" STAGE4_IMPORTED_PROTOC_REAL)
file(REAL_PATH "${STAGE4_PROTOC_EXECUTABLE}" STAGE4_REQUIRED_PROTOC_REAL)
if(NOT STAGE4_IMPORTED_PROTOC_REAL STREQUAL STAGE4_REQUIRED_PROTOC_REAL)
  message(FATAL_ERROR "Protobuf config protoc differs from STAGE4_PROTOC_EXECUTABLE")
endif()
execute_process(
  COMMAND "${STAGE4_REQUIRED_PROTOC_REAL}" --version
  RESULT_VARIABLE STAGE4_PROTOC_RESULT
  OUTPUT_VARIABLE STAGE4_PROTOC_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT STAGE4_PROTOC_RESULT EQUAL 0 OR NOT STAGE4_PROTOC_VERSION STREQUAL "libprotoc 33.6")
  message(FATAL_ERROR "Phase-0 requires libprotoc 33.6")
endif()

set(V2_PROTO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../proto")
set(V2_PROTO "${V2_PROTO_DIR}/slope_sim_interfaces_v2.proto")
set(V2_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${V2_GENERATED_DIR}")
add_library(stage4_v2_proto STATIC "${V2_PROTO}")
target_link_libraries(stage4_v2_proto PUBLIC protobuf::libprotobuf)
target_include_directories(stage4_v2_proto PUBLIC "${V2_GENERATED_DIR}")
protobuf_generate(
  TARGET stage4_v2_proto
  LANGUAGE cpp
  IMPORT_DIRS "${V2_PROTO_DIR}"
  PROTOC_OUT_DIR "${V2_GENERATED_DIR}"
)

add_library(
  slope_sim_client STATIC
  "${CMAKE_CURRENT_LIST_DIR}/../client/command_instance_lock.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/command_twist.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/atomic_segment.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/command_lease.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/raw_v2_payload.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/raw_wheel_command.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/interactive_recorder_window.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/recorder_queue.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/recorder_session.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/mcap_session_reader.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/mcap_session_writer.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/replay_plan.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/ros_sensor_pair.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../client/v2_topics.cpp"
)
target_include_directories(
  slope_sim_client
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/../client/include"
    "${STAGE4_MCAP_INCLUDE_DIR}"
)
target_compile_definitions(slope_sim_client PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
target_link_libraries(slope_sim_client PRIVATE stage4_v2_proto OpenSSL::Crypto)
add_executable(
  slope_sim_client_topic_contract_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/v2_topics_test.cpp"
)
target_link_libraries(slope_sim_client_topic_contract_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_topic_contract COMMAND slope_sim_client_topic_contract_test)
add_executable(
  slope_sim_client_command_lease_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/command_lease_test.cpp"
)
target_link_libraries(slope_sim_client_command_lease_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_command_lease COMMAND slope_sim_client_command_lease_test)
add_executable(
  slope_sim_client_command_twist_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/command_twist_test.cpp"
)
target_link_libraries(slope_sim_client_command_twist_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_command_twist COMMAND slope_sim_client_command_twist_test)
add_executable(
  slope_sim_client_command_socket_frame_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/command_socket_frame_test.cpp"
)
target_link_libraries(slope_sim_client_command_socket_frame_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_command_socket_frame COMMAND slope_sim_client_command_socket_frame_test)
add_executable(
  slope_sim_client_interactive_socket_stop_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/interactive_socket_stop_test.cpp"
)
target_link_libraries(
  slope_sim_client_interactive_socket_stop_test
  PRIVATE slope_sim_client stage4_v2_proto eCAL::core OpenSSL::Crypto
)
add_test(NAME slope_sim_client_interactive_socket_stop COMMAND slope_sim_client_interactive_socket_stop_test)
add_executable(
  slope_sim_client_command_lock_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/command_lock_test.cpp"
)
target_link_libraries(slope_sim_client_command_lock_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_command_lock COMMAND slope_sim_client_command_lock_test)
add_executable(
  slope_sim_client_recorder_queue_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/recorder_queue_test.cpp"
)
target_link_libraries(slope_sim_client_recorder_queue_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_recorder_queue COMMAND slope_sim_client_recorder_queue_test)
add_executable(
  slope_sim_client_atomic_segment_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/atomic_segment_test.cpp"
)
target_link_libraries(slope_sim_client_atomic_segment_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_atomic_segment COMMAND slope_sim_client_atomic_segment_test)
add_executable(
  slope_sim_client_mcap_session_writer_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/mcap_session_writer_test.cpp"
)
target_link_libraries(slope_sim_client_mcap_session_writer_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_mcap_session_writer COMMAND slope_sim_client_mcap_session_writer_test)
add_executable(
  slope_sim_client_mcap_session_reader_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/mcap_session_reader_test.cpp"
)
target_link_libraries(slope_sim_client_mcap_session_reader_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_mcap_session_reader COMMAND slope_sim_client_mcap_session_reader_test)
add_executable(
  slope_sim_client_replay_plan_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/replay_plan_test.cpp"
)
target_link_libraries(slope_sim_client_replay_plan_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_replay_plan COMMAND slope_sim_client_replay_plan_test)
add_executable(
  slope_sim_client_recorder_session_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/recorder_session_test.cpp"
)
target_link_libraries(slope_sim_client_recorder_session_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_recorder_session COMMAND slope_sim_client_recorder_session_test)
add_executable(
  slope_sim_client_interactive_recorder_window_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/interactive_recorder_window_test.cpp"
)
target_link_libraries(slope_sim_client_interactive_recorder_window_test PRIVATE slope_sim_client)
add_test(NAME slope_sim_client_interactive_recorder_window COMMAND slope_sim_client_interactive_recorder_window_test)
add_executable(
  slope_sim_client_validate_wheel_command
  "${CMAKE_CURRENT_LIST_DIR}/../client/validate_wheel_command.cpp"
)
target_link_libraries(slope_sim_client_validate_wheel_command PRIVATE slope_sim_client OpenSSL::Crypto)
add_executable(
  slope_sim_client_validate_v2_payload
  "${CMAKE_CURRENT_LIST_DIR}/../client/validate_v2_payload.cpp"
)
target_link_libraries(slope_sim_client_validate_v2_payload PRIVATE slope_sim_client OpenSSL::Crypto)
add_executable(
  slope_sim_stage4_command
  "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_command.cpp"
)
target_link_libraries(slope_sim_stage4_command PRIVATE slope_sim_client stage4_v2_proto eCAL::core OpenSSL::Crypto)
add_executable(
  slope_sim_stage4_subscriber
  "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_subscriber.cpp"
)
target_link_libraries(slope_sim_stage4_subscriber PRIVATE slope_sim_client eCAL::core OpenSSL::Crypto)
add_executable(
  slope_sim_stage4_recorder
  "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_recorder.cpp"
)
target_link_libraries(slope_sim_stage4_recorder PRIVATE slope_sim_client stage4_v2_proto eCAL::core OpenSSL::Crypto)
add_executable(
  slope_sim_stage4_replay
  "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_replay.cpp"
)
target_link_libraries(slope_sim_stage4_replay PRIVATE slope_sim_client eCAL::core OpenSSL::Crypto)
add_executable(
  slope_sim_client_replay_integration_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/stage4_replay_integration_test.cpp"
)
target_compile_definitions(
  slope_sim_client_replay_integration_test
  PRIVATE STAGE4_REPLAY_EXECUTABLE="$<TARGET_FILE:slope_sim_stage4_replay>"
)
target_link_libraries(slope_sim_client_replay_integration_test PRIVATE slope_sim_client eCAL::core OpenSSL::Crypto)
add_test(NAME slope_sim_client_replay_integration COMMAND slope_sim_client_replay_integration_test)
add_executable(
  slope_sim_stage4_export
  "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_export.cpp"
)
target_link_libraries(slope_sim_stage4_export PRIVATE slope_sim_client stage4_v2_proto OpenSSL::Crypto)
add_executable(
  slope_sim_client_export_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/stage4_export_test.cpp"
)
target_compile_definitions(
  slope_sim_client_export_test
  PRIVATE STAGE4_EXPORT_EXECUTABLE="$<TARGET_FILE:slope_sim_stage4_export>"
)
add_dependencies(slope_sim_client_export_test slope_sim_stage4_export)
target_link_libraries(slope_sim_client_export_test PRIVATE slope_sim_client OpenSSL::Crypto)
add_test(NAME slope_sim_client_export COMMAND slope_sim_client_export_test)

# 阶段四 E：release 只交付公共 SDK 与五个生产 participant，不安装测试或 probe。
install(TARGETS slope_sim_client ARCHIVE DESTINATION lib)
install(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../client/include/" DESTINATION include)
install(
  TARGETS
    slope_sim_stage4_command
    slope_sim_stage4_subscriber
    slope_sim_stage4_recorder
    slope_sim_stage4_replay
    slope_sim_stage4_export
  RUNTIME DESTINATION bin
)

add_executable(
  slope_sim_client_ros_sensor_pair_test
  "${CMAKE_CURRENT_LIST_DIR}/../client/tests/ros_sensor_pair_test.cpp"
)
target_link_libraries(slope_sim_client_ros_sensor_pair_test PRIVATE slope_sim_client stage4_v2_proto)
add_test(NAME slope_sim_client_ros_sensor_pair COMMAND slope_sim_client_ros_sensor_pair_test)

if(STAGE4_WITH_ROS)
  # Jazzy 的二进制 package 只导出 Release；阶段四仍统一使用 RelWithDebInfo。
  set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
  if(NOT DEFINED STAGE4_LIVOX_MSG_SOURCE)
    message(FATAL_ERROR "Phase-0 ROS Bridge requires STAGE4_LIVOX_MSG_SOURCE")
  endif()
  if(NOT DEFINED STAGE4_ROSIDL_PYTHON_EXECUTABLE OR
     NOT EXISTS "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}")
    message(FATAL_ERROR "Phase-0 ROS Bridge requires STAGE4_ROSIDL_PYTHON_EXECUTABLE")
  endif()
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  # ROSIDL 属于 Jazzy 的 Python 3.12 环境；上方 Phase-0 CTest 已固定使用 runtime Python 3.10。
  set(Python3_EXECUTABLE "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}")
  set(Python3_EXECUTABLE "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}" CACHE FILEPATH
      "Python interpreter used by the ROSIDL generator" FORCE)
  # ament_package_xml() resolves the already-created imported target, not the variable above.
  set_property(TARGET Python3::Interpreter PROPERTY IMPORTED_LOCATION
               "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../livox_ros_driver2_interfaces"
                   "${CMAKE_CURRENT_BINARY_DIR}/livox_ros_driver2_interfaces")
  add_executable(
    slope_sim_stage4_ros2_bridge
    "${CMAKE_CURRENT_LIST_DIR}/../client/stage4_ros2_bridge.cpp"
  )
  target_link_libraries(slope_sim_stage4_ros2_bridge PRIVATE slope_sim_client stage4_v2_proto eCAL::core OpenSSL::Crypto geometry_msgs::geometry_msgs rclcpp::rclcpp sensor_msgs::sensor_msgs rosgraph_msgs::rosgraph_msgs tf2_ros::tf2_ros stage4_livox_ros2_interfaces)
  install(TARGETS slope_sim_stage4_ros2_bridge RUNTIME DESTINATION bin)
  install(
    FILES "${CMAKE_CURRENT_LIST_DIR}/../client/rviz/stage4_live.rviz"
    DESTINATION cpp/client/rviz
  )
  add_executable(
    slope_sim_client_ros2_bridge_integration_test
    "${CMAKE_CURRENT_LIST_DIR}/../client/tests/stage4_ros2_bridge_integration_test.cpp"
  )
  target_compile_definitions(
    slope_sim_client_ros2_bridge_integration_test
    PRIVATE STAGE4_ROS2_BRIDGE_EXECUTABLE="$<TARGET_FILE:slope_sim_stage4_ros2_bridge>"
  )
  add_dependencies(slope_sim_client_ros2_bridge_integration_test slope_sim_stage4_ros2_bridge)
  target_link_libraries(
    slope_sim_client_ros2_bridge_integration_test
    PRIVATE slope_sim_client stage4_v2_proto eCAL::core geometry_msgs::geometry_msgs rclcpp::rclcpp sensor_msgs::sensor_msgs rosgraph_msgs::rosgraph_msgs tf2_msgs::tf2_msgs stage4_livox_ros2_interfaces
  )
  add_test(NAME slope_sim_client_ros2_bridge_integration COMMAND slope_sim_client_ros2_bridge_integration_test)
  set_tests_properties(
    slope_sim_client_ros2_bridge_integration
    PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}/livox_ros_driver2_interfaces:$ENV{LD_LIBRARY_PATH}"
  )
  add_executable(
    slope_sim_client_ros2_replay_bridge_integration_test
    "${CMAKE_CURRENT_LIST_DIR}/../client/tests/stage4_ros2_bridge_integration_test.cpp"
  )
  target_compile_definitions(
    slope_sim_client_ros2_replay_bridge_integration_test
    PRIVATE STAGE4_ROS2_BRIDGE_EXECUTABLE="$<TARGET_FILE:slope_sim_stage4_ros2_bridge>" STAGE4_ROS2_BRIDGE_REPLAY_TEST=1
  )
  add_dependencies(slope_sim_client_ros2_replay_bridge_integration_test slope_sim_stage4_ros2_bridge)
  target_link_libraries(
    slope_sim_client_ros2_replay_bridge_integration_test
    PRIVATE slope_sim_client stage4_v2_proto eCAL::core geometry_msgs::geometry_msgs rclcpp::rclcpp sensor_msgs::sensor_msgs rosgraph_msgs::rosgraph_msgs tf2_msgs::tf2_msgs stage4_livox_ros2_interfaces
  )
  add_test(NAME slope_sim_client_ros2_replay_bridge_integration COMMAND slope_sim_client_ros2_replay_bridge_integration_test)
  set_tests_properties(
    slope_sim_client_ros2_replay_bridge_integration
    PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}/livox_ros_driver2_interfaces:$ENV{LD_LIBRARY_PATH}"
  )
endif()

add_executable(ecal_v2_raw_probe ecal_v2_raw_probe.cpp sha256.hpp)
target_link_libraries(ecal_v2_raw_probe PRIVATE slope_sim_client eCAL::core stage4_v2_proto OpenSSL::Crypto)
# dry-run 不初始化 eCAL，但仍要加载冻结的 raw eCAL ABI，避免链接器裁掉该边界。
target_link_options(ecal_v2_raw_probe PRIVATE "-Wl,--no-as-needed")
add_executable(v2_golden v2_golden.cpp sha256.hpp)
target_link_libraries(v2_golden PRIVATE stage4_v2_proto OpenSSL::Crypto)
