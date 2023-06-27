cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS visualization)

## Include directories
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

## Add executable targets
add_executable(camera src/camera.cpp)
add_executable(lidar src/lidar.cpp)

## Link libraries
target_link_libraries(camera ${catkin_LIBRARIES})
target_link_libraries(lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Install the executables
install(TARGETS camera lidar
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Add car_node target
add_executable(car_node src/car.cpp)
target_link_libraries(car_node ${catkin_LIBRARIES})
install(TARGETS car_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
