cmake_minimum_required(VERSION 2.8.3)
project(image_to_rviz)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
  roslib
)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/params.cfg
  #...
)

catkin_package()


# make sure configure headers are built before any node using them

add_executable(image_to_rviz_node src/image_to_rviz_node.cpp)
add_dependencies(image_to_rviz_node ${PROJECT_NAME}_gencfg)
target_link_libraries(image_to_rviz_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vins_image_to_rviz src/vins_image_to_rviz.cpp)
add_dependencies(vins_image_to_rviz ${PROJECT_NAME}_gencfg)
target_link_libraries(vins_image_to_rviz ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vins_fusion_outdoor_img src/vins_fusion_outdoor_img.cpp)
add_dependencies(vins_fusion_outdoor_img ${PROJECT_NAME}_gencfg)
target_link_libraries(vins_fusion_outdoor_img ${catkin_LIBRARIES} ${OpenCV_LIBS})
