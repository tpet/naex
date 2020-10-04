cmake_minimum_required(VERSION 3.0)
project(naex)

add_compile_options(-std=c++11)

find_package(Boost COMPONENTS graph REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(PkgConfig)
pkg_check_modules(Flann REQUIRED)
include_directories(${Flann_INCLUDE_DIR})
link_directories(${Flann_LIBDIR})

pkg_check_modules(LZ4 REQUIRED)
include_directories(${LZ4_INCLUDE_DIRS})
link_directories(${LZ4_LIBRARY_DIR})

include_directories(include)

# Catkin
set(CATKIN_DEPS
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_ros
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS ${CATKIN_DEPS}
)

add_executable(follower src/follower_node.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})

add_executable(planner src/planner_node.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} Eigen3::Eigen ${Flann_LIBRARY} lz4)

install(
    TARGETS
        follower
        planner
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
