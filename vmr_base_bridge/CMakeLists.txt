cmake_minimum_required(VERSION 3.16)
project(vmr_base_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(VMR_SDK_LIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib" CACHE PATH "Path to packaged VMR SDK shared libraries")
set(VMR_SDK_ARCH "" CACHE STRING "Vendor SDK architecture directory override: x86 or arm64")
set(VMR_SDK_UBUNTU_VERSION "" CACHE STRING "Vendor SDK Ubuntu version directory override, such as 22.04 or 24.04")

if(NOT VMR_SDK_UBUNTU_VERSION)
  set(DETECTED_UBUNTU_VERSION "")
  if(EXISTS "/etc/os-release")
    file(STRINGS "/etc/os-release" OS_RELEASE_VERSION_ID REGEX "^VERSION_ID=")
    if(OS_RELEASE_VERSION_ID)
      string(REGEX REPLACE "^VERSION_ID=\"?([0-9.]+)\"?$" "\\1" DETECTED_UBUNTU_VERSION "${OS_RELEASE_VERSION_ID}")
    endif()
  endif()
  if(DETECTED_UBUNTU_VERSION)
    set(VMR_SDK_UBUNTU_VERSION "${DETECTED_UBUNTU_VERSION}")
  else()
    set(VMR_SDK_UBUNTU_VERSION "22.04")
  endif()
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/VmrSDKApi.h")
  message(FATAL_ERROR "Required SDK header not found: ${CMAKE_CURRENT_SOURCE_DIR}/include/VmrSDKApi.h")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/VmrDefine.h")
  message(FATAL_ERROR "Required SDK header not found: ${CMAKE_CURRENT_SOURCE_DIR}/include/VmrDefine.h")
endif()

if(NOT VMR_SDK_ARCH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(VMR_SDK_ARCH "arm64")
  else()
    set(VMR_SDK_ARCH "x86")
  endif()
endif()

set(VMR_SDK_LIBRARY_DIR "${VMR_SDK_LIB_ROOT}/${VMR_SDK_ARCH}/${VMR_SDK_UBUNTU_VERSION}")
set(VMR_SDK_LIBRARY "${VMR_SDK_LIBRARY_DIR}/libvmr_sdk.so")
if(NOT EXISTS "${VMR_SDK_LIBRARY}")
  message(FATAL_ERROR "VMR SDK library not found: ${VMR_SDK_LIBRARY}")
endif()
message(STATUS "Using VMR SDK library: ${VMR_SDK_LIBRARY}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VmrLocation.msg"
  "msg/VmrMoveStatus.msg"
  "msg/VmrTimingDiagnostic.msg"
  "srv/StepMove.srv"
  "srv/VectorMove.srv"
  "srv/NavTarget.srv"
  "srv/CancelTask.srv"
  "srv/QueryTaskStatus.srv"
  "srv/ControlRelay.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(vmr_base_bridge_lib src/vmr_base_bridge.cpp)
target_include_directories(vmr_base_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vmr_base_bridge_lib
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  yaml_cpp_vendor
)
target_link_libraries(vmr_base_bridge_lib ${cpp_typesupport_target} ${VMR_SDK_LIBRARY} yaml-cpp)
set_target_properties(vmr_base_bridge_lib PROPERTIES
  BUILD_RPATH "${VMR_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(vmr_base_bridge_node src/vmr_base_bridge_node.cpp)
target_link_libraries(vmr_base_bridge_node vmr_base_bridge_lib)
ament_target_dependencies(vmr_base_bridge_node rclcpp)
set_target_properties(vmr_base_bridge_node PROPERTIES
  BUILD_RPATH "${VMR_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${VMR_SDK_LIBRARY}
  DESTINATION lib
)

install(
  PROGRAMS
    scripts/right_triangle_trajectory.py
    scripts/static_map_publisher.py
    scripts/task_mode_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS vmr_base_bridge_lib vmr_base_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 yaml_cpp_vendor rosidl_default_runtime)

ament_package()
