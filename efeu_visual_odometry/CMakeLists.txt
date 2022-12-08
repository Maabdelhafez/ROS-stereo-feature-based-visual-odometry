cmake_minimum_required(VERSION 3.0.2)
project(efeu_visual_odometry)

set (CMAKE_CXX_STANDARD 11)


#set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)




find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	image_transport
	cv_bridge
  )
find_package(OpenCV REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4/ NO_DEFAULT_PATH) 

#	"/usr/include/eigen3/"
#	FMT REQUIRED







catkin_package(
 INCLUDE_DIRS include
 LIBRARIES efeu_visual_odometry
 CATKIN_DEPENDS roscpp std_msgs  geometry_msgs image_transport
 DEPENDS system_lib include_directories  
)




 







###########
## Build ##
###########


 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


########################
#### points_node #######
########################

add_executable(points src/points.cpp)
target_link_libraries(points ${catkin_LIBRARIES})
target_link_libraries(points ${OpenCV_LIBRARIES})
target_link_libraries(points ${image_transport_LIBRARIES})

#add_dependencies(points efeu_visual_odometry_cpp)



add_executable(depth src/depth.cpp)
target_link_libraries(depth ${catkin_LIBRARIES})
target_link_libraries(depth ${OpenCV_LIBRARIES})
target_link_libraries(depth ${image_transport_LIBRARIES})


#add_dependencies(depth efeu_visual_odometry_cpp)




add_executable(repubLeftCam src/repubLeftCam.cpp)
target_link_libraries(repubLeftCam ${catkin_LIBRARIES})
target_link_libraries(repubLeftCam ${OpenCV_LIBRARIES})
target_link_libraries(repubLeftCam ${image_transport_LIBRARIES})


#add_dependencies(depth efeu_visual_odometry_cpp)

   
   
   
add_executable(repubRightCam src/repubRightCam.cpp)
target_link_libraries(repubRightCam ${catkin_LIBRARIES})
target_link_libraries(repubRightCam ${OpenCV_LIBRARIES})
target_link_libraries(repubRightCam ${image_transport_LIBRARIES})


#add_dependencies(depth efeu_visual_odometry_cpp)

   
   
 
   
add_executable(lpub src/lpub.cpp)
target_link_libraries(lpub ${catkin_LIBRARIES})
target_link_libraries(lpub ${OpenCV_LIBRARIES})
target_link_libraries(lpub ${image_transport_LIBRARIES})


#add_dependencies(depth efeu_visual_odometry_cpp)

   
   
   
   
add_executable(rpub src/rpub.cpp)
target_link_libraries(rpub ${catkin_LIBRARIES})
target_link_libraries(rpub ${OpenCV_LIBRARIES})
target_link_libraries(rpub ${image_transport_LIBRARIES})


#add_dependencies(depth efeu_visual_odometry_cpp)
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   




