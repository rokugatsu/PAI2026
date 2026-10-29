cmake_minimum_required(VERSION 3.14)
project(rtx)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io registration surface)
find_package(pcl_conversions REQUIRED PATHs /opt/ros/humble)

find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED system)

find_package(pcl_ros)
find_package(geometry_msgs)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(message_filters REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(OpenMP REQUIRED)

find_package(robo_msgs REQUIRED)
#find_package(jsk_rviz_plugins REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

set(msg_files
  "msg/Vehicle.msg"
  "msg/Waypoint.msg"
  "msg/TrafficLight.msg"
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  "/opt/ros/humble/include/tf2_eigen/"
  "/opt/ros/humble/include/tf2/"
  "/opt/ros/humble/include/tf2_ros/"
  "/opt/ros/humble/include/tf2_msgs/"
  "/opt/ros/humble/include/tf2_geometry_msgs/"
  "/opt/ros/humble/include/tf2_eigen/tf2_eigen/"
  "/opt/ros/humble/include/nav_msgs/"
  "/opt/ros/humble/include/visualization_msgs/"
  #"/opt/ros/humble/include/vision_msgs/"
  #"/opt/ros/humble/include/rviz_common/"
  "#/opt/ros/humble/opt/rviz_ogre_vendor/include/OGRE/"
  #"/home/june/colcon_ws/src/ndt_omp_ros2/include/"
  #"/home/june/colcon_ws/src/robo/src/common"
  "/home/june/colcon_ws/src/common/"
  #"/home/june/colcon_ws/install/robo_msgs/include/robo_msgs/robo_msgs"

)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

#message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
#message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
#message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_library(ndt_omp
#  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/gicp_omp.cpp
#  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/ndt_omp.cpp
#  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/voxel_grid_covariance_omp.cpp
#)

#############################
## ROS 2 multigrid ndt_omp ##
#############################
#ament_auto_add_library(multigrid_ndt_omp SHARED
#  src/multigrid_pclomp/multi_voxel_grid_covariance_omp.cpp
#  src/multigrid_pclomp/multigrid_ndt_omp.cpp
#)
#target_link_libraries(multigrid_ndt_omp ${PCL_LIBRARIES})
#if(OpenMP_CXX_FOUND)
#  target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
#else()
#  message(WARNING "OpenMP not found")
#endif()

add_executable(waypoint_planner_node src/waypoint_planner/waypoint_planner_node.cpp src/waypoint_planner/waypoint_planner_function.hpp src/waypoint_planner/wp_search_freespace.cpp src/waypoint_planner/waypoint_planner_AreaMap.cpp /home/june/colcon_ws/src/rtx/src/common/common_function.cpp)
ament_target_dependencies(waypoint_planner_node rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros visualization_msgs robo_msgs pcl_msgs vision_msgs)
target_link_libraries(waypoint_planner_node ${PCL_LIBRARIES} pcl_segmentation)

add_executable(motion_coordinator_node src/motion_coordinator/motion_coordinator_node.cpp src/motion_coordinator/motion_controller_function.cpp src/motion_coordinator/motion_coordinator_controll.cpp src/motion_coordinator/motion_coordinator_planner.cpp /home/june/colcon_ws/src/rtx/src/common/common_function.cpp)
ament_target_dependencies(motion_coordinator_node rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros visualization_msgs robo_msgs pcl_msgs rviz_2d_overlay_msgs)
target_link_libraries(motion_coordinator_node ${PCL_LIBRARIES})

add_executable(rpi_udp_node src/rpi_udp/rpi_udp_node.cpp)
ament_target_dependencies(rpi_udp_node rclcpp robo_msgs)
target_link_libraries(rpi_udp_node)

add_executable(localizer_node src/localizer/Localizer_node.cpp src/localizer/geo_pos_conv.cpp src/localizer/Localizer_AreaMapAccessor.cpp)
ament_target_dependencies(localizer_node rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions robo_msgs tf2_ros visualization_msgs pcl_msgs tf2_sensor_msgs)
target_link_libraries(localizer_node ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} pcl_segmentation)

add_executable(loadsurface_detector_node src/loadsurface_detector/plane_extraction_node.cpp)
ament_target_dependencies(loadsurface_detector_node rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions robo_msgs tf2_ros visualization_msgs pcl_msgs tf2_sensor_msgs)
target_link_libraries(loadsurface_detector_node ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} pcl_segmentation)

add_executable(pointcloud_map_transformer_node src/pointcloud_map_transformer_node/pointcloud_map_transformer_node.cpp)
ament_target_dependencies(pointcloud_map_transformer_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs)

add_executable(polygon_map_transformer_node src/polygon_map_transformer_node/polygon_map_transformer_node.cpp)
ament_target_dependencies(polygon_map_transformer_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS
    #waypoint_server_node
    waypoint_planner_node
    motion_coordinator_node
    rpi_udp_node
    localizer_node
    loadsurface_detector_node
    pointcloud_map_transformer_node
    polygon_map_transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# install headers
install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

ament_package()
