cmake_minimum_required(VERSION 3.10)
project(mocap4r2_optitrack_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# NatNet protocol, client and fake server: no ROS, no proprietary SDK.
add_library(natnet_protocol SHARED
  src/natnet_protocol.cpp
  src/natnet_client.cpp
  src/fake_natnet_server.cpp
)
set_target_properties(natnet_protocol PROPERTIES OUTPUT_NAME mocap4r2_natnet)
target_include_directories(natnet_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
find_package(Threads REQUIRED)
target_link_libraries(natnet_protocol PUBLIC Threads::Threads)

# Optional official NatNet SDK (not distributed: install it with install_natnet_sdk.sh).
option(MOCAP4R2_USE_NATNET_SDK "Build the backend on the official NatNet SDK if it is found" ON)
set(NATNET_SDK_DIR "$ENV{NATNET_SDK_DIR}" CACHE PATH "Directory of the NatNet SDK")
if(NOT NATNET_SDK_DIR)
  set(NATNET_SDK_DIR "$ENV{HOME}/.local/share/natnet_sdk")
endif()
set(NATNET_SDK_FOUND FALSE)
if(MOCAP4R2_USE_NATNET_SDK)
  find_path(NATNET_INCLUDE_DIR NatNetClient.h PATHS "${NATNET_SDK_DIR}/include" NO_DEFAULT_PATH)
  find_library(NATNET_LIBRARY NatNet PATHS "${NATNET_SDK_DIR}/lib" NO_DEFAULT_PATH)
  if(NATNET_INCLUDE_DIR AND NATNET_LIBRARY)
    set(NATNET_SDK_FOUND TRUE)
  endif()
endif()

set(driver_sources src/frame_converter.cpp src/optitrack_driver.cpp src/natnet_backend.cpp)
if(NATNET_SDK_FOUND)
  message(STATUS "NatNet SDK found: ${NATNET_LIBRARY} (backends: sdk, open)")
  list(APPEND driver_sources src/natnet_sdk_backend.cpp)
else()
  message(STATUS "NatNet SDK not found in ${NATNET_SDK_DIR} (backend: open)")
endif()

add_library(${PROJECT_NAME} SHARED ${driver_sources})
if(NATNET_SDK_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MOCAP4R2_HAVE_NATNET_SDK)
  target_include_directories(${PROJECT_NAME} PRIVATE "${NATNET_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} PRIVATE "${NATNET_LIBRARY}")
  get_filename_component(NATNET_LIB_DIR "${NATNET_LIBRARY}" DIRECTORY)
  set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${NATNET_LIB_DIR}")
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC
  natnet_protocol
  mocap4r2_control::mocap4r2_control
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_optitrack_driver::OptitrackDriver")

add_executable(optitrack_driver src/optitrack_driver_main.cpp)
target_link_libraries(optitrack_driver ${PROJECT_NAME})

add_executable(natnet_simulator src/natnet_simulator.cpp)
target_link_libraries(natnet_simulator natnet_protocol)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/install_natnet_sdk.sh DESTINATION lib/${PROJECT_NAME})
install(TARGETS natnet_protocol ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS optitrack_driver natnet_simulator RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_natnet_protocol test/test_natnet_protocol.cpp)
  target_link_libraries(test_natnet_protocol natnet_protocol)
  ament_add_gtest(test_frame_converter test/test_frame_converter.cpp)
  target_link_libraries(test_frame_converter ${PROJECT_NAME})
  ament_add_gtest(test_optitrack_driver test/test_optitrack_driver.cpp TIMEOUT 180)
  target_link_libraries(test_optitrack_driver ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mocap4r2_control mocap4r2_interfaces rclcpp)
ament_package()
