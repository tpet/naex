cmake_minimum_required(VERSION 3.9)
project(naex)

add_compile_options(-std=c++11)

find_package(Boost COMPONENTS graph REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(OpenMP REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(eigen REQUIRED eigen3)
include_directories(${eigen_INCLUDE_DIRS})
link_directories(${eigen_LIBRARY_DIRS})

pkg_check_modules(flann REQUIRED flann)
include_directories(${flann_INCLUDE_DIRS})
link_directories(${flann_LIBRARY_DIRS})

pkg_check_modules(lz4 REQUIRED liblz4)
include_directories(${lz4_INCLUDE_DIRS})
link_directories(${lz4_LIBRARY_DIRS})

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

#add_executable(follower src/follower_node.cpp)
#target_link_libraries(follower ${catkin_LIBRARIES})

add_executable(lidar_model src/lidar_model.cpp)
target_link_libraries(lidar_model ${catkin_LIBRARIES} ${eigen_LIBRARIES} ${lz4_LIBRARIES} OpenMP::OpenMP_CXX)

# https://stackoverflow.com/a/51448364/3456661
add_executable(planner src/planner_node.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${eigen_LIBRARIES} ${flann_LIBRARIES} ${lz4_LIBRARIES} OpenMP::OpenMP_CXX)
#target_link_libraries(planner PRIVATE Eigen3::Eigen)
#target_link_libraries(planner ${catkin_LIBRARIES} ${Flann_LIBRARY} lz4 OpenMP::OpenMP_CXX)

add_executable(incremental_flann_index src/incremental_flann_index.cpp)
target_link_libraries(incremental_flann_index ${eigen_LIBRARIES} ${Flann_LIBRARY} lz4 OpenMP::OpenMP_CXX)

add_executable(integral_radius_search src/integral_radius_search.cpp)
target_link_libraries(integral_radius_search ${eigen_LIBRARIES} ${Flann_LIBRARY}  ${lz4_LIBRARIES} OpenMP::OpenMP_CXX)

install(
    TARGETS
#        follower
        planner
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        scripts/get_plan
        scripts/path_follower
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
