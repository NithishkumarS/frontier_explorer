cmake_minimum_required(VERSION 2.8.3)
project(frontier_explorer)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  geometry_msgs
  actionlib
  move_base
  move_base_msgs
)

catkin_package(
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(frontier_explorer src/BFS.cpp src/explore.cpp src/map.cpp src/main.cpp src/sensor.cpp)
target_link_libraries(frontier_explorer ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
  testfrontierExplorer
  test/frontierExplorerTest.launch
  test/main.cpp
  test/testMapGen.cpp
  test/testBFS.cpp
  test/testSensor.cpp

  #src/main.cpp
  src/BFS.cpp 
  #src/explore.cpp 
  src/map.cpp 
  src/sensor.cpp
  )
  target_link_libraries(
  testfrontierExplorer
  ${catkin_LIBRARIES}
  )
  add_dependencies(
  testfrontierExplorer
  frontier_explorer
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  )
endif()

