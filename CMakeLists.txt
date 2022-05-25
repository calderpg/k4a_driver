cmake_minimum_required(VERSION 3.9)
project(k4a_driver)

find_package(ament_cmake REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
find_package(k4a REQUIRED)

ament_export_dependencies(k4a)
ament_export_dependencies(image_transport)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(rclcpp)

## Build options
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)

add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/k4a_driver.hpp
            src/${PROJECT_NAME}/k4a_driver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
                      k4a::k4a
                      rclcpp::rclcpp
                      ${sensor_msgs_TARGETS}
                      ${tf2_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      image_transport::image_transport)
target_link_libraries(${PROJECT_NAME} PRIVATE
                      OpenMP::OpenMP_CXX
                      Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} PUBLIC
                        cxx_std_11)

add_executable(k4a_driver_node src/k4a_driver_node.cpp)
target_link_libraries(k4a_driver_node
                      ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} k4a_driver_node EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_package()