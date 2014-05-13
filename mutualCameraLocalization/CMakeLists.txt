cmake_minimum_required(VERSION 2.4.6)
project(mutual_camera_localization)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  dynamic_reconfigure
  geometry_msgs
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# Add opencv
find_package(OpenCV REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/MonocularPoseEstimator.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport dynamic_reconfigure geometry_msgs
  DEPENDS eigen OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the executable
add_executable(mutual_camera_localization 
  src/mutual_camera_localization_node.cpp 
  src/led_detector.cpp 
  src/visualization.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(mutual_camera_localization ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

# link the executable to the necessary libs
target_link_libraries(mutual_camera_localization
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )