cmake_minimum_required(VERSION 2.8.3)
project(k4a_driver)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)
find_package(k4a REQUIRED)

catkin_package(CATKIN_DEPENDS
               roscpp
               sensor_msgs
               geometry_msgs
               DEPENDS
               Eigen3)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${Eigen3_INCLUDE_DIRS}
                           ${k4a_INCLUDE_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)

## It's not clear if add_compile_options does the right things for flags that
## may differ between languages and target type.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

add_executable(k4a_driver_node src/k4a_driver_node.cpp)
add_dependencies(k4a_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(k4a_driver_node ${catkin_LIBRARIES} k4a::k4a)

install(TARGETS k4a_driver_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
