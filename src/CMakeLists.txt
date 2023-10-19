cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

# 设置Boost的根目录路径
set(GTSAM_ROOT "/home/tonywang/code/gtsam-4.0.0/build")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  serial
  message_filters
)

find_package( GTSAMCMakeTools )

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  serial
)

# include directories
include_directories(
	include
  include/iir
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${GLOB_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)


  file(GLOB GLOG_LIBRARIES /usr/lib/x86_64-linux-gnu/libglog.so)
	set(GLOG_INCLUDE_DIRS /usr/include)
	message("glog_include_dirs=${GLOG_INCLUDE_DIRS}")
	message("glog_libraries=${GLOG_LIBRARIES}")
  set(LIBSRC
  include/iir/Biquad.cpp
  include/iir/Butterworth.cpp
  include/iir/Cascade.cpp
  include/iir/ChebyshevI.cpp
  include/iir/ChebyshevII.cpp
  include/iir/Custom.cpp
  include/iir/PoleFilter.cpp
  include/iir/RBJ.cpp
  include/iir/State.cpp)

set(LIBINCLUDE
  include/iir/Biquad.h
  include/iir/Butterworth.h
  include/iir/Cascade.h
  include/iir/ChebyshevI.h
  include/iir/ChebyshevII.h
  include/iir/Common.h
  include/iir/Custom.h
  include/iir/Layout.h
  include/iir/MathSupplement.h
  include/iir/PoleFilter.h
  include/iir/RBJ.h
  include/iir/State.h
  include/iir/Types.h)

  add_library(iir
  SHARED
  ${LIBSRC}
  )


###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${GLOG_LIBRARIES})

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam ${GLOG_LIBRARIES})

# GNSS2XYZ
add_executable(${PROJECT_NAME}_gps2xyz src/gps_to_xyz_wzm.cpp)
target_link_libraries(${PROJECT_NAME}_gps2xyz ${catkin_LIBRARIES} )

# add_executable(${PROJECT_NAME}_pose_estimate src/pose_estimate.cpp src/Mahony_AHRS.cpp src/EKF_AHRS.cpp src/Convert.cpp src/Madgwick_AHRS.cpp)
# target_link_libraries(${PROJECT_NAME}_pose_estimate
# 	iir
# 	${catkin_LIBRARIES}
# )
