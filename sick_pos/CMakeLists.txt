cmake_minimum_required(VERSION 2.4.6)
project(sick_pose)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  laser_geometry
  tf
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)


# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/sickPose.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  dynamic_reconfigure 
  geometry_msgs 
  laser_geometry 
  tf
  DEPENDS eigen
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# make the executable
add_executable(sick_pose
  src/sick_pose_node.cpp 
  include/sick_pose/sick_pose_node.h
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(sick_pose ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(sick_pose
   yaml-cpp
   ${catkin_LIBRARIES}
 )
