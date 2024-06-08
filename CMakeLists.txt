
cmake_minimum_required(VERSION 3.5)
project(wall_ground_plane_extractor_from_pointcloud)

# Can we use C++17 in indigo?
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
# set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  fast_gicp
  geodesy
  geometry_msgs
  interactive_markers
  message_generation
  ndt_omp
  nmea_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdl_graph_slam_nodelet
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nmea_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable (plane_extraction src/plane_extraction.cpp)
target_link_libraries (plane_extraction         
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
  )

add_executable (plane_extraction_with_out_clustering src/plane_extraction_with_out_clustering.cpp)
target_link_libraries (plane_extraction_with_out_clustering          
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
  )


  add_executable (plane_extraction_using_selected_roi src/plane_extraction_using_selected_roi.cpp)
  target_link_libraries (plane_extraction_using_selected_roi         
      ${PCL_LIBRARIES}
      ${catkin_LIBRARIES}
    )







