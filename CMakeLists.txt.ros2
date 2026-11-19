cmake_minimum_required(VERSION 3.28...4.2)
project(k4a_driver VERSION 0.0.0 LANGUAGES CXX)

include(CheckIPOSupported)
include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(k4a REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Defaults when this project is built on its own, as colcon does. A parent
# project or the command line can override them.
if(PROJECT_IS_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  endif()
  if(NOT DEFINED CMAKE_COMPILE_WARNING_AS_ERROR)
    set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
  endif()
  if(NOT DEFINED CMAKE_INTERPROCEDURAL_OPTIMIZATION)
    check_ipo_supported(RESULT ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ${ipo_supported})
  endif()
endif()

# Warnings for this project's own targets only; consumers don't inherit them.
add_library(k4a_driver_warnings INTERFACE)
set(k4a_driver_warning_flags
  -Wall -Wextra -Wconversion -Wshadow -Wold-style-cast -Wpessimizing-move
  -Wuninitialized -Werror=non-virtual-dtor)
target_compile_options(k4a_driver_warnings INTERFACE
  "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:${k4a_driver_warning_flags}>")

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/k4a_driver.ros2.cpp)
target_sources(${PROJECT_NAME}
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/k4a_driver/k4a_driver.ros2.hpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${geometry_msgs_TARGETS}
    image_transport::image_transport
    k4a::k4a
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
  PRIVATE
    Eigen3::Eigen
    $<BUILD_INTERFACE:k4a_driver_warnings>)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.ros2.cpp)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE ${PROJECT_NAME} k4a_driver_warnings)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  image_transport
  k4a
  rclcpp
  sensor_msgs
  tf2_msgs)
ament_package()
