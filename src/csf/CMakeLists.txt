cmake_minimum_required(VERSION 3.0.2)
project(csf)

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

find_package(PCL REQUIRED)

add_subdirectory(src)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pcl_ros
  DEPENDS PCL
)

add_executable(${PROJECT_NAME}_node main.cpp mycsf.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  CSF
)