cmake_minimum_required(VERSION 2.8.3)
project(moveit_sample)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules moveit_planners_ompl)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(moveit_sample src/moveit_sample.cpp)
target_link_libraries(moveit_sample ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_sample DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
